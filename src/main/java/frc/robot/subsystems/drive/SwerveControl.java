package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.DriveConstants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlip;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import org.littletonrobotics.junction.Logger;

public class SwerveControl {
  public static DoubleFunction<Translation2d> quadBezier(
      Translation2d start, Translation2d end, Translation2d c) {
    return (t) -> {
      var p0 = start.interpolate(c, t);
      var p1 = c.interpolate(end, t);
      return p0.interpolate(p1, t);
    };
  }

	public static DoubleFunction<Translation2d> quadBezierPrime(Translation2d p0, Translation2d p1, Translation2d p2) {
		var b0 = p1.minus(p0).times(2);
		var b1 = p2.minus(p1).times(2);

		return (t) -> {
			return b0.interpolate(b1, t);

		};
	}

  public static DoubleFunction<Translation2d> cubicBezier(
      Translation2d start, Translation2d end, Translation2d c1, Translation2d c2) {
    return (t) -> {
      var b1 = start.interpolate(c1, t);
      var b2 = c1.interpolate(c2, t);
      var b3 = c2.interpolate(end, t);

      return quadBezier(b1, b3, b2).apply(t);
    };
  }

	public static DoubleFunction<Translation2d> cubicBezierPrime(Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3) {
		var b0 = p1.minus(p0).times(3);
		var b1 = p2.minus(p1).times(3);
		var b2 = p3.minus(p2).times(3);

		return quadBezier(b0, b2, b1);
	}

	public static DoubleFunction<Translation2d> cubicBezierDoublePrime(Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3) {
		var b0 = p1.minus(p0).times(3);
		var b1 = p2.minus(p1).times(3);
		var b2 = p3.minus(p2).times(3);

		return quadBezierPrime(b0, b1, b2);
	}

	public static DoubleFunction<Double> rotationalCubicBezier(Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3) {
		final var q = cubicBezierPrime(p0, p1, p2, p3);
		return (t) -> {
			var tangent = q.apply(t);
			return Math.atan2(tangent.getY(), tangent.getX());
		};
	}

	public static DoubleFunction<Double> omegaCubicBezier(Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3) {
		final var Bprime = cubicBezierPrime(p0, p1, p2, p3);
		final var Bdoubleprime = cubicBezierDoublePrime(p0, p1, p2, p3);

		return (t) -> {
			Translation2d d1 = Bprime.apply(t);
			Translation2d d2 = Bdoubleprime.apply(t);

			double x = d1.getX(), y = d1.getY();
			double x2 = d1.getY(), y2 = d2.getY();

			return (x * y2 - y * x2) / (x*x + y*y);
		};
	}

	public static DoubleFunction<Double> alphaCubicBezier(Translation2d p0, Translation2d p1, Translation2d p2, Translation2d p3) {
    final var Bprime = cubicBezierPrime(p0, p1, p2, p3);
    final var BdoublePrime = cubicBezierDoublePrime(p0, p1, p2, p3);
    // B''' for cubic Bézier is constant
    Translation2d BtriplePrime = p3.minus(p2.times(3)).plus(p1.times(3)).minus(p0);

    return (t) -> {
        Translation2d d1 = Bprime.apply(t);
        Translation2d d2 = BdoublePrime.apply(t);
        Translation2d d3 = BtriplePrime;

        double x = d1.getX(), y = d1.getY();
        double x2 = d2.getX(), y2 = d2.getY();
        double x3 = d3.getX(), y3 = d3.getY();

        double denom = x*x + y*y;
        double num = (x*x3 + y*y3) * denom - 2 * (x*x2 + y*y2) * (x*x2 + y*y2);

        return num / (denom * denom);
    };
	}

  public record NativeHolonomicTrajectory(List<Vector<N3>> q, List<Vector<N3>> qPrime, List<Vector<N3>> qPrimePrime) {}

  public record PathState(Translation2d point, ChassisSpeeds velocity) {
    public Translation2d unitDirection() {
      var speed = Math.hypot(velocity().vxMetersPerSecond, velocity().vyMetersPerSecond);
      var d = new Translation2d(velocity().vxMetersPerSecond, velocity().vyMetersPerSecond);
      if (speed > 1e-6) {
        d = d.div(speed);
      }
      return d;
    }

    public double linearSpeed() {
      return Math.hypot(velocity().vxMetersPerSecond, velocity().vyMetersPerSecond);
    }
  }
  ;

  private static Translation2d safeNormalize(Translation2d vector) {
    double mag = vector.getNorm();
    if (mag > 1e-6) {
      vector = vector.div(mag);
    }
    return vector;
  }

  private static Translation2d extendedBisector(
      Translation2d p1, Translation2d p2, Translation2d corner, double k, boolean invert) {
    var d1 = safeNormalize(corner.minus(p1));
    var d2 = safeNormalize(p2.minus(corner));
    var tangent = d1.plus(d2);
    var bisector = safeNormalize(new Translation2d(tangent.getY(), -tangent.getX()));
    if (invert) {
      bisector = bisector.unaryMinus();
    }
    return corner.plus(bisector.times(k));
  }

  public static Translation2d[] cubicStateConstrainedControls(
      PathState start, PathState end) {
    var p0 = start.point();
    var p3 = end.point();
    var distance = p0.getDistance(p3);
    var Beta = distance / 3.0;

		var s1 = start.linearSpeed();
		var s2 = end.linearSpeed();

    var v1 = start.unitDirection();
    var v2 = end.unitDirection();

    var p1 = p0.plus(v1.times(Beta * s1 / DriveConstants.maxDriveSpeedMps));
    var p2 = p3.minus(v2.times(Beta * s2 / DriveConstants.maxDriveSpeedMps));

    return new Translation2d[] { p0, p1, p2, p3 };
  }

  public static NativeHolonomicTrajectory smoothTrenchTraverseCurve(
      PathState start, PathState end, int samples) {
    var cornerCloseRel =
        new Translation2d(FieldConstants.LinesVertical.starting, FieldConstants.RightTrench.depth);
    var cornerFarRel =
        cornerCloseRel.plus(new Translation2d(FieldConstants.RightTrench.width, 0.0));
    var trenchCenterRel =
        cornerCloseRel.plus(
            new Translation2d(
                FieldConstants.RightTrench.width / 2.0, -FieldConstants.RightTrench.depth / 2.0));

    var cornerClose = AllianceFlip.apply(cornerCloseRel);
    var cornerFar = AllianceFlip.apply(cornerFarRel);
    var trenchCenter = AllianceFlip.apply(trenchCenterRel);

    double kk = FieldConstants.RightTrench.depth / 2.0;
    var B1 = extendedBisector(start.point(), cornerFar, cornerClose, kk, false);
    var B2 = extendedBisector(cornerClose, end.point(), cornerFar, kk, false);

    double gamma = 0.8 * DriveConstants.maxDriveSpeedMps;
    var vEst1 = safeNormalize(trenchCenter.minus(B1)).times(gamma);
    var vEst2 = safeNormalize(B2.minus(trenchCenter)).times(gamma);

    var m1 = B1.plus(trenchCenter.minus(B1));
    var m2 = B2.plus(trenchCenter.minus(B2));

    var points1 =
        cubicStateConstrainedControls(
            start, new PathState(B1, new ChassisSpeeds(vEst1.getX(), vEst1.getY(), 0.0)));
    var points2 = new Translation2d[] { B1, m1, m2, B2 };
    var points3 =
        cubicStateConstrainedControls(
            new PathState(B2, new ChassisSpeeds(vEst2.getX(), vEst2.getY(), 0.0)), end);

    return joinAndDifferentiateCubics(new Translation2d[][] { points1, points2, points3 }, samples);
  }

	public static NativeHolonomicTrajectory joinAndDifferentiateCubics(Translation2d[][] controlPoints, int N) {
		List<DoubleFunction<Translation2d>> qs = new ArrayList<>();
		List<DoubleFunction<Translation2d>> qps = new ArrayList<>();
		List<DoubleFunction<Translation2d>> qpps = new ArrayList<>();

		List<DoubleFunction<Double>> qrs = new ArrayList<>();
		List<DoubleFunction<Double>> qrps = new ArrayList<>();
		List<DoubleFunction<Double>> qrpps = new ArrayList<>();

		for (var cps : controlPoints) {
			qs.add(cubicBezier(cps[0], cps[3], cps[1], cps[2]));
			qps.add(cubicBezierPrime(cps[0], cps[1], cps[2], cps[3]));
			qpps.add(cubicBezierDoublePrime(cps[0], cps[1], cps[2], cps[3]));

			qrs.add(rotationalCubicBezier(cps[0], cps[1], cps[2], cps[3]));
			qrps.add(omegaCubicBezier(cps[0], cps[1], cps[2], cps[3]));
			qrpps.add(alphaCubicBezier(cps[0], cps[1], cps[2], cps[3]));
		}

		return new NativeHolonomicTrajectory(
			concatenateToStateVector(joinAndSampleCurves(qs, N), joinAndSampleCurves(qrs, N)),
			concatenateToStateVector(joinAndSampleCurves(qps, N), joinAndSampleCurves(qrps, N)),
			concatenateToStateVector(joinAndSampleCurves(qpps, N), joinAndSampleCurves(qrpps, N))
		);
	}

	// Set up the scan range for backward pass
	private static final double scanRes = 0.001;
	private static final int scanLength = 5000;
	private static final double[] scanRange = new double[scanLength];

	static {
			// Populate scanRange with alternating positive and negative increasing offsets
			for (int i = 1; i < scanLength; i++) {
					scanRange[i] = (i % 2 == 0) ? scanRes * Math.floor(i / 2.0) : scanRes * Math.ceil(-i / 2.0);
			}
	}

	// Determines maximum velocity at each discrete step to be able to reach next point 
	// based on robot physicsal constraints
	public static Optional<double[]> optimalBackwardsPass(NativeHolonomicTrajectory trajectory, ChassisSpeeds endSpeeds, Vector<N3> mdiag, double Fmax) {
		int N = trajectory.q().size();
		double[] xMax = new double[N];
		xMax[N-1]=Math.hypot(endSpeeds.vxMetersPerSecond, endSpeeds.vyMetersPerSecond);
		for (int i=N-2; i>=0; i--) {
			double dx = trajectory.q().get(i+1).get(0)-trajectory.q().get(i).get(0);
			double dy = trajectory.q().get(i+1).get(1)-trajectory.q().get(i).get(1);
			double ds = dx*dx + dy*dy;

			// |F_i|^2 = |b + au|^2 <= F_max
			// |b + au|^2 = b^2 + 2aub + a^2u^2
			//
			// a^2u^2 + 2aub + b^2 = 0
			// Quadratic formula:
			// 	A = a^2
			// 	B = 2ab
			// 	C = B^2
			var a = trajectory.qPrime().get(i).elementTimes(mdiag);
			var b = trajectory.qPrimePrime().get(i).elementTimes(mdiag);

			double A = a.elementPower(2).elementSum();
			double B = 2 * a.elementTimes(b).elementSum();
			double C = b.elementPower(2).elementSum()  - Fmax * Fmax;
			
			double D = B*B - 4*A*C;

			if (D < 0) return Optional.empty();

			double uMin = (-B - Math.sqrt(D)) / (2*A);

			// v2^2 = v1^2 + 2ad
			// v1 = sqrt(v2^2 - 2ad)
			double x_i = Math.sqrt(xMax[i+1] - 2 * uMin * ds);
			if (Double.isNaN(x_i) || x_i < 0) return Optional.empty();
			xMax[i]=x_i;
		}

		return Optional.of(xMax);
	}

	public static List<Vector<N3>> concatenateToStateVector(List<Translation2d> p, List<Double> r) {
		assert p.size() == r.size();
		List<Vector<N3>> vectors = new ArrayList<>();
		for (int i=0; i<p.size(); ++i) {
			vectors.add(VecBuilder.fill(p.get(i).getX(), p.get(i).getY(), r.get(i)));
		}
		return vectors;
	}

	public static <T> List<T> joinAndSampleCurves(List<DoubleFunction<T>> curves, int N) {
		int samplesPerCurve = N / curves.size();
		List<T> points = new ArrayList<>();
		for (var q : curves) {
			points.addAll(sampleContinousCurve(q, samplesPerCurve));
		}
		return points;
	}

  public static <T> List<T> sampleContinousCurve(
      DoubleFunction<T> q, int samples) {
    List<T> points = new ArrayList<>(samples);
    for (int i = 0; i < samples; ++i) {
      double t = (double) i / (samples - 1);
      points.add(q.apply(t));
    }
    return points;
  }

  public static void testTrajectory() {
    var corner = new Translation2d(FieldConstants.LinesVertical.starting, 1.194 / 2.0);
    var s1 =
        new PathState(
            RobotState.getInstance().getSimulatedDrivePose().getTranslation(),
            RobotState.getInstance().getFieldVelocity());
    var s2 =
        new PathState(corner.plus(new Translation2d(4.0, 1.5)), new ChassisSpeeds(0.0, 3.0, 0.0));
    var traj = smoothTrenchTraverseCurve(s1, s2, 100);

    Logger.recordOutput(
        "Testing/Trajectory",
        traj.q().stream()
            .map(v -> new Transform3d(new Translation3d(v.get(0), v.get(1), 0.0), new Rotation3d()))
            .toArray(Transform3d[]::new));
  }
}
