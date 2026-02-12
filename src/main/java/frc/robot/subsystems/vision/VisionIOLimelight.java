package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private final CameraConfig config;
  private final NetworkTable nt;
  private boolean throttling = false;
  private double lastHeartbeat = 0.0;

  public VisionIOLimelight(CameraConfig config) {
    this.config = config;
    nt = NetworkTableInstance.getDefault().getTable(config.name());
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void setRewindEnabled(boolean enabled) {
    LimelightHelpers.setRewindEnabled(config.name(), enabled);
  }

  @Override
  public void recordRewind(int seconds) {
    LimelightHelpers.triggerRewindCapture(config.name(), seconds);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // IMU / Throttle set
    if (DriverStation.isDisabled() && !throttling) {
      LimelightHelpers.SetThrottle(config.name(), 200);
      LimelightHelpers.SetIMUMode(config.name(), 1);
      throttling = true;
    } else if (DriverStation.isEnabled() && throttling) {
      LimelightHelpers.SetThrottle(config.name(), 0);
      LimelightHelpers.SetIMUMode(config.name(), VisionConstants.matchImuMode);
      throttling = false;
    }

    // Update megatag pose
    LimelightHelpers.SetRobotOrientation(
        config.name(),
        RobotState.getInstance().getEstimatedPose().getRotation().getDegrees(),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);

    double heartbeat = LimelightHelpers.getHeartbeat(config.name());
    inputs.connected = lastHeartbeat - heartbeat <= 2;
    lastHeartbeat = heartbeat;

    if (!LimelightHelpers.getTV(config.name())) return;
    var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name());
    if (megatag != null) {
      var stdDevs = nt.getEntry("stddevs").getDoubleArray(new double[12]);
      inputs.poseEstimate =
          new MultitagPoseEstimate(
              megatag.pose,
              megatag.avgTagDist,
              megatag.tagCount,
              VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[5]),
              megatag.timestampSeconds);
    }
  }
}
