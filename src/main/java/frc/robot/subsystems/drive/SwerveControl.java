package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleFunction;
import org.littletonrobotics.junction.Logger;

public class SwerveControl {
  public record RobotPathState(Pose2d pose, ChassisSpeeds speeds) {}

  private static double tangentTime = 0.3;

  // TODO: convert these to generate lists of points with a requested sample size instead of
  // functions

  public static DoubleFunction<Translation2d> quadBezier(
      Translation2d start, Translation2d end, Translation2d c) {
    return (t) -> {
      var p0 = start.interpolate(c, t);
      var p1 = c.interpolate(end, t);
      return p0.interpolate(p1, t);
    };
  }

  public static DoubleFunction<Translation2d> cubicBezier(
      Translation2d start, Translation2d end, Translation2d c1, Translation2d c2) {
    return (t) -> {
      var b1 = start.interpolate(c1, t);
      var b2 = c1.interpolate(c2, t);
      var b3 = c2.interpolate(end, t);

      return quadBezier(b1, b2, b3).apply(t);
    };
  }

  public static DoubleFunction<Translation2d> cubicBezierPrime(
      Translation2d start, Translation2d end, Translation2d c1, Translation2d c2) {
    return (t) -> {
      var b1 = c1.minus(start).times(3);
      var b2 = c2.minus(c1).times(3);
      var b3 = end.minus(c2).times(3);

      return quadBezier(b1, b2, b3).apply(t);
    };
  }

  public static DoubleFunction<Translation2d> cubicBezierStateConstrained(
      RobotPathState start, RobotPathState end) {
    Translation2d c1 =
        new Translation2d(start.speeds().vxMetersPerSecond, start.speeds().vyMetersPerSecond)
            .rotateBy(start.pose().getRotation())
            .times(tangentTime);
    Translation2d c2 =
        new Translation2d(end.speeds().vxMetersPerSecond, end.speeds().vyMetersPerSecond)
            .rotateBy(end.pose().getRotation())
            .times(tangentTime);

    return cubicBezier(
        start.pose().getTranslation(),
        end.pose().getTranslation(),
        start.pose().getTranslation().plus(c1),
        end.pose().getTranslation().minus(c2));
  }

  public static List<Translation2d> bezierPath(List<RobotPathState> W, int samples_per_segment) {
    int segments = W.size() - 1;
    List<Translation2d> path = new ArrayList<>(samples_per_segment * segments);
    int k = 0;
    int N = samples_per_segment * segments;
    for (int i = 0; i < W.size() - 1; ++i) {
      var q = cubicBezierStateConstrained(W.get(i), W.get(i + 1));
      for (int j = 0; j < samples_per_segment; ++j) {
        double t = (double) k / (N - 1);
        path.add(q.apply(t));
        k += 1;
      }
    }
    return path;
  }

  public static void timeOptimalBackwardsPass() {}

  public static void testTrajectory() {
    var state1 =
        new RobotPathState(RobotState.getInstance().getSimulatedDrivePose(), new ChassisSpeeds());
    var state2 =
        new RobotPathState(
            new Pose2d(1.0, 1.0, Rotation2d.kZero), new ChassisSpeeds(4.0, 0.0, 0.0));
    List<Transform3d> trajectory =
        Arrays.stream(
                SwerveControl.bezierPath(List.of(state1, state2), 20).toArray(Translation2d[]::new))
            .map(t -> new Transform3d(new Translation3d(t), Rotation3d.kZero))
            .toList();
    Logger.recordOutput("Testing/bezierTrajectory", trajectory.toArray(Transform3d[]::new));
  }
}
