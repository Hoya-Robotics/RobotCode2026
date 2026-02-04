package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState.HubObservation;
import org.littletonrobotics.junction.AutoLog;

public interface LocalizationCameraIO {
  @AutoLog
  public static class LocalizationInputs {
    public boolean isConnected = false;

    public boolean hubInView = false;
    public HubObservation hubObservation =
        new HubObservation(new Transform3d(), new Transform3d(), -1);

    public MultitagPoseEstimate[] globalPoseEstimates = new MultitagPoseEstimate[] {};
  }

  public record MultitagPoseEstimate(
      double timestamp,
      Pose3d pose,
      double avgTagDistance,
      int tagCount,
      double quality,
      double[] stdDevs) {}

  public default CameraConfig getConfig() {
    return null;
  }

  public default void enableRewind(boolean enable) {}

  public default void captureRewind(int duration) {}

  public default void updateInputs(LocalizationInputs input) {}
}
