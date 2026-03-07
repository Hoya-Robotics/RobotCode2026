package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {
  private static final double[] kdefaultStddevs = new double[12];

  private final Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier;
  private final CameraConfig config;
  private final NetworkTable NT;
  private double heartBeat = 0.0;

  public VisionIOLimelight(
      CameraConfig config, Optional<Supplier<Pose3d>> dynamicCameraPoseSupplier) {
    this.dynamicCameraPoseSupplier = dynamicCameraPoseSupplier;
    this.config = config;
    this.NT = NetworkTableInstance.getDefault().getTable(config.name());

    LimelightHelpers.setRewindEnabled(config.name(), VisionConstants.rewindEnabled);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double lastHeartbeat = heartBeat;
    heartBeat = LimelightHelpers.getHeartbeat(config.name());
    inputs.isConnected = heartBeat != lastHeartbeat;

    if (dynamicCameraPoseSupplier.isPresent()) {
      Pose3d camPose = dynamicCameraPoseSupplier.get().get();
      LimelightHelpers.setCameraPose_RobotSpace(
          config.name(),
          camPose.getX(),
          camPose.getY(),
          camPose.getZ(),
          camPose.getRotation().getX(),
          camPose.getRotation().getY(),
          camPose.getRotation().getZ());
    }

    Rotation2d gyroYaw = RobotState.getInstance().getEstimatedPose().getRotation();
    LimelightHelpers.SetRobotOrientation(
        config.name(), gyroYaw.getRadians(), 0.0, 0.0, 0.0, 0.0, 0.0);

    LimelightHelpers.PoseEstimate mt2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name());
    inputs.avgTagDist = mt2Estimate.avgTagDist;
    inputs.poseEstimate = mt2Estimate.pose;
    inputs.timestamp = mt2Estimate.timestampSeconds;
    inputs.numTags = mt2Estimate.tagCount;

    double[] rawStdDevs = this.NT.getEntry("stddevs").getDoubleArray(kdefaultStddevs);
    inputs.stdDevs = VecBuilder.fill(rawStdDevs[6], rawStdDevs[7], rawStdDevs[11]);

    int tid = (int) this.NT.getEntry("tid").getInteger(0);
    inputs.hubInSight = VisionConstants.hubTags.contains(tid);
    if (inputs.hubInSight) {
      inputs.primaryTid = tid;
      inputs.robotToHub = LimelightHelpers.getCameraPose3d_RobotSpace(config.name());
    }
  }

  @Override
  public CameraConfig getConfig() {
    return this.config;
  }

  @Override
  public void captureRewind(double duration) {
    LimelightHelpers.triggerRewindCapture(config.name(), duration);
  }
}
