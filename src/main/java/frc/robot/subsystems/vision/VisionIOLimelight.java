package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private final CameraConfig config;
  private final NetworkTable NT;
  private static final double[] kDefaultStddevs = new double[12];

  public VisionIOLimelight(CameraConfig config) {
    this.config = config;
    this.NT = NetworkTableInstance.getDefault().getTable(config.name());

    LimelightHelpers.setRewindEnabled(config.name(), true);
    setRobotToCamera(config.robotToCamera());
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(config.name(), 1);
      LimelightHelpers.SetThrottle(config.name(), 1000);
    } else {
      LimelightHelpers.SetIMUMode(config.name(), 3);
      LimelightHelpers.SetThrottle(config.name(), 0);
    }

    inputs.seesTarget = NT.getEntry("tv").getDouble(0.0) == 1.0;
    if (!inputs.seesTarget) return;

    try {
      inputs.pose3d = LimelightHelpers.getBotPose3d_wpiBlue(config.name());
      var ll_mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name());
      var ll_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name());

      if (ll_mt1 != null) {
        inputs.mt1 = MultitagEstimate.fromLL(ll_mt1);
        inputs.fiducials = FiducialObservation.arrayFromLL(ll_mt1.rawFiducials);
      }
      if (ll_mt2 != null) {
        inputs.mt2 = MultitagEstimate.fromLL(ll_mt2);
      }

      inputs.stddevs = NT.getEntry("stddevs").getDoubleArray(kDefaultStddevs);
    } catch (Exception e) {
      DriverStation.reportWarning("Vision update failed: " + e.getMessage(), false);
    }
  }

  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {
    LimelightHelpers.setCameraPose_RobotSpace(
        config.name(),
        robotToCamera.getX(),
        robotToCamera.getY(),
        robotToCamera.getZ(),
        robotToCamera.getRotation().getMeasureX().in(Degrees),
        robotToCamera.getRotation().getMeasureY().in(Degrees),
        robotToCamera.getRotation().getMeasureZ().in(Degrees));
  }

  @Override
  public void setRobotOrientation(Rotation2d gyroYaw) {
    LimelightHelpers.SetRobotOrientation(
        config.name(), gyroYaw.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void captureRewind(double duration) {
    LimelightHelpers.triggerRewindCapture(config.name(), duration);
  }
}
