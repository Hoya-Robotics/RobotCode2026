package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.LimelightOutputs;
import org.littletonrobotics.junction.AutoLog;

public interface LocalizationCameraIO {
  @AutoLog
  public static class LocalizationInputs {
    public boolean isConnected = false;

    public boolean hubInView = false;
    public double cameraToHubTimestamp = 0.0;
    public Transform3d cameraToHub = new Transform3d();

    public MT2Observation[] globalPoseObservations;
  }

  public record MT2Observation(
      double timestamp, Pose3d pose, double avgTagDistance, int tagCount) {}

  public static class LocalizationOutputs {
    public LimelightOutputs llOutputs;
  }

  default void updateInputs(LocalizationInputs input) {}

  default void applyOutputs(LocalizationOutputs outputs) {}
}
