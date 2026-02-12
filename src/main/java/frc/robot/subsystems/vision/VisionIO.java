package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotConfig.CameraConfig;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public MultitagPoseEstimate poseEstimate =
        new MultitagPoseEstimate(Pose2d.kZero, 0.0, 0, VecBuilder.fill(0.0, 0.0, 0.0), 0.0);
  }

  public record MultitagPoseEstimate(
      Pose2d pose, double avgTagDist, int tags, Vector<N3> stdDevs, double timestamp) {}

  public default void updateInputs(VisionIOInputs inputs) {}

  public default CameraConfig getConfig() {
    return null;
  }
}
