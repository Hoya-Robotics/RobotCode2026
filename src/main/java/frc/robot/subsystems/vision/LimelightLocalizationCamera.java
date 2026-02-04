package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState.*;
import frc.robot.util.LimelightHelpers;
import java.util.function.Supplier;

public class LimelightLocalizationCamera implements LocalizationCameraIO {
  private final CameraConfig config;

  private final Supplier<Rotation2d> externalYawSupplier;
  private final NetworkTable table;

  private boolean disabledState = false;
  private double lastHeartbeat = 0.0;

  public LimelightLocalizationCamera(
      CameraConfig config, Supplier<Rotation2d> externalYawSupplier) {
    this.config = config;
    this.externalYawSupplier = externalYawSupplier;

    table = NetworkTableInstance.getDefault().getTable(config.name());
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void updateInputs(LocalizationInputs inputs) {
    NetworkTableInstance.getDefault().flush();

    double heartbeat = LimelightHelpers.getHeartbeat(config.name());
    inputs.isConnected = heartbeat - lastHeartbeat < 2;
    lastHeartbeat = heartbeat;

    LimelightHelpers.SetRobotOrientation(
        config.name(), externalYawSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

    if (DriverStation.isDisabled() && !disabledState) {
      disabledState = true;
      LimelightHelpers.SetThrottle(config.name(), 200);
      LimelightHelpers.SetIMUMode(config.name(), 1);
    } else if (disabledState) {
      disabledState = false;
      LimelightHelpers.SetThrottle(config.name(), 0);
      LimelightHelpers.SetIMUMode(config.name(), VisionConstants.matchImuMode);
    }

    // No tag in view
    if (table.getEntry("tv").getDouble(0.0) != 1.0) {
      return;
    }

    int tid = (int) table.getEntry("tid").getDouble(0.0);

    // Check for hub tags
    if (tid == 9 || tid == 10 || tid == 25 || tid == 26) {
      inputs.hubInView = true;
      var cameraToHub = LimelightHelpers.getCameraPose3d_TargetSpace(config.name());
      inputs.hubObservation =
          new HubObservation(
              config.robotToCamera(),
              new Transform3d(cameraToHub.getTranslation(), cameraToHub.getRotation()),
              tid);
    } else {
      inputs.hubInView = false;
    }

    var mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name());
    if (mt2Estimate != null) {
      var stdDevs = table.getEntry("stddevs").getDoubleArray(new double[] {0.3, 0.3, 10});
      inputs.globalPoseEstimates =
          new MultitagPoseEstimate[] {
            new MultitagPoseEstimate(
                mt2Estimate.timestampSeconds,
                new Pose3d(mt2Estimate.pose),
                mt2Estimate.avgTagDist,
                mt2Estimate.tagCount,
                1.0,
                stdDevs)
          };
    }
  }

  @Override
  public void enableRewind(boolean enable) {
    LimelightHelpers.setRewindEnabled(config.name(), enable);
  }

  @Override
  public void captureRewind(int duration) {
    LimelightHelpers.triggerRewindCapture(config.name(), duration);
  }
}
