package frc.robot.subsystems.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotConfig.CameraConfig;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public MultitagPoseEstimate poseEstimate = null;
  }

  public record MultitagPoseEstimate(
      Pose2d pose, double avgTagDist, int tags, Vector<N3> stdDevs, double timestamp) {}

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void recordRewind(int seconds) {}

  public default void setRewindEnabled(boolean enabled) {}

  public default CameraConfig getConfig() {
    return null;
  }
}
