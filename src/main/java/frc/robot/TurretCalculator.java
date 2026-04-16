package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.AllianceFlip;
import org.littletonrobotics.junction.Logger;

public class TurretCalculator {

  private static final int kMaxIterations = 10;
  private static final double kConvergenceEpsilon = 0.001;

  // Cached constant — avoids Translation3d.toTranslation2d() allocation every loop
  private static final Translation2d kTurretOffset =
      TurretConstants.robotToTurret.getTranslation().toTranslation2d();

  private static final InterpolatingDoubleTreeMap passingTofMap = new InterpolatingDoubleTreeMap();

  static {
    passingTofMap.put(4.0, 1.25);
    passingTofMap.put(5.0, 1.27);
    passingTofMap.put(6.0, 1.28);
  }

  public static double getTOF(double distance, boolean passing) {
    return passing ? passingTofMap.get(distance) : 0.196 * distance + 0.33;
  }

  public record TurretParameters(
      Angle azimuthAngle,
      AngularVelocity azimuthVelocity,
      Angle hoodAngle,
      AngularVelocity flywheelSpeed) {

    private static Angle computeHood(double x, boolean passing) {
      return Degrees.of(passing ? -0.233 * x * x + 4.52 * x + 8.21 : 1.92 * x + 7.28);
    }

    private static AngularVelocity computeFlywheel(double x, boolean passing) {
      return RotationsPerSecond.of(passing ? 5.0 * x + 14.5 : 3.6 * x + 28.0);
    }

    private static Angle computeAzimuth(Angle angle, Angle currentAngle) {
      double targetRotations = angle.in(Rotations);
      double currentRotations = currentAngle.in(Rotations);
      double diff = targetRotations - currentRotations;
      diff = MathUtil.inputModulus(diff, -0.5, 0.5);
      double closestTarget = currentRotations + diff;
      if (closestTarget
          > TurretConstants.maxAzimuthAngle.in(Rotations) - Units.degreesToRotations(3.0)) {
        closestTarget -= 1;
      } else if (closestTarget
          < TurretConstants.minAzimuthAngle.in(Rotations) + Units.degreesToRotations(3.0)) {
        closestTarget += 1;
      }
      return Rotations.of(closestTarget);
    }

    public static TurretParameters compute(
        double distance,
        Angle targetAzimuthAngle,
        Angle currentAzimuthAngle,
        AngularVelocity azimuthFF,
        boolean passing) {
      return new TurretParameters(
          computeAzimuth(targetAzimuthAngle, currentAzimuthAngle),
          azimuthFF,
          computeHood(distance, passing),
          computeFlywheel(distance, passing));
    }
  }

  public static Translation2d getPassingTarget() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Distance flippedY = AllianceFlip.apply(robotPose).getMeasureY();
    boolean mirror = flippedY.gt(FieldConstants.fieldWidth.div(2.0));
    return new Translation2d(
        FieldConstants.passingX,
        mirror
            ? FieldConstants.fieldWidth.minus(FieldConstants.passingY)
            : FieldConstants.passingY);
  }

  public static TurretParameters turretIterativeMovingSetpoint(
      Translation2d target, boolean passing, Angle currentAzimuthAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    robotPose =
        robotPose.exp(RobotState.getInstance().getFieldVelocity().toTwist2d(0.003)); // phase shift
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();

    // Inline rigidPointVelocity + rotateBy as raw doubles — eliminates 2 Translation2d allocations
    double cosYaw = robotPose.getRotation().getCos();
    double sinYaw = robotPose.getRotation().getSin();
    ChassisSpeeds robotVel = RobotState.getInstance().getRobotVelocity();
    double rvx = robotVel.vxMetersPerSecond - robotVel.omegaRadiansPerSecond * kTurretOffset.getY();
    double rvy = robotVel.vyMetersPerSecond + robotVel.omegaRadiansPerSecond * kTurretOffset.getX();
    double fvx = rvx * cosYaw - rvy * sinYaw;
    double fvy = rvx * sinYaw + rvy * cosYaw;

    double turretX = turretPose.getX();
    double turretY = turretPose.getY();
    double targetX = target.getX();
    double targetY = target.getY();

    // Iterative convergence with raw doubles — no Translation2d allocations in loop
    double dx = targetX - turretX;
    double dy = targetY - turretY;
    double distance = Math.sqrt(dx * dx + dy * dy);
    double tof = getTOF(distance, passing);
    for (int i = 0; i < kMaxIterations; ++i) {
      dx = targetX - (turretX + fvx * tof);
      dy = targetY - (turretY + fvy * tof);
      double newDist = Math.sqrt(dx * dx + dy * dy);
      if (Math.abs(newDist - distance) < kConvergenceEpsilon) break;
      distance = newDist;
      tof = getTOF(distance, passing);
    }

    // Raw double aim vector — no Translation2d or Rotation2d allocations
    double aimDx = targetX - (turretX + fvx * tof);
    double aimDy = targetY - (turretY + fvy * tof);
    double squaredNorm = aimDx * aimDx + aimDy * aimDy;
    double azimuthRad = Math.atan2(aimDy, aimDx) - robotPose.getRotation().getRadians();
    double cross2d = aimDy * fvx - aimDx * fvy;
    double azVelRPS =
        (cross2d / squaredNorm) - RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;

    Logger.recordOutput("TurretCalculator/targetDistance", distance);
    return TurretParameters.compute(
        distance,
        Radians.of(azimuthRad),
        currentAzimuthAngle,
        RotationsPerSecond.of(azVelRPS),
        passing);
  }
}
