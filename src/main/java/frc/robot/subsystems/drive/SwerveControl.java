package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleFunction;
import org.littletonrobotics.junction.Logger;

public class SwerveControl {
  public record RobotPathState(Pose2d pose, ChassisSpeeds speeds) {}

  private static double tangentTime = 1.0 / 3;

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

  public static void timeOptimalBackwardsPass() {}

  public static void testTrajectory() {
    var p1 = RobotState.getInstance().getSimulatedDrivePose().getTranslation();
    var p2 = new Translation2d(4.5, 0.5);

    var length = p1.getDistance(p2);

    double delta = p2.minus(p1).getX();
    var c = p2.minus(new Translation2d(delta * 2, 0.0));

    var q = quadBezier(p1, p2, c);
    List<Translation2d> points = new ArrayList<>();
    for (int i = 0; i < 20; ++i) {
      points.add(q.apply((double) i / 19));
    }

    Logger.recordOutput(
        "Testing/Trajectory",
        points.stream()
            .map(p -> new Transform3d(new Translation3d(p), new Rotation3d()))
            .toArray(Transform3d[]::new));
  }
}
