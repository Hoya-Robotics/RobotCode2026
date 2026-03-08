package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {
  private static final double[] kdefaultStddevs =
      new double[] {0, 0, 0, 0, 0, 0, 0.5, 0.5, 0, 0, 0, 0.1};

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
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetThrottle(config.name(), 200);
      LimelightHelpers.SetIMUMode(config.name(), 1);
    } else {
      LimelightHelpers.SetThrottle(config.name(), 0);
      LimelightHelpers.SetIMUMode(config.name(), 4);
    }

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
          Math.toDegrees(camPose.getRotation().getX()),
          Math.toDegrees(camPose.getRotation().getY()),
          Math.toDegrees(camPose.getRotation().getZ()));
    }

    Rotation2d gyroYaw = RobotState.getInstance().getEstimatedPose().getRotation();
    LimelightHelpers.SetRobotOrientation(
        config.name(), gyroYaw.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

    /*
    LimelightHelpers.PoseEstimate mt2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name());*/
    LimelightHelpers.PoseEstimate mt2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name());
    inputs.avgTagDist = mt2Estimate.avgTagDist;
    inputs.poseEstimate = mt2Estimate.pose;
    inputs.timestamp = mt2Estimate.timestampSeconds;
    inputs.numTags = mt2Estimate.tagCount;

    double[] rawStdDevs = this.NT.getEntry("stddevs").getDoubleArray(kdefaultStddevs);
    boolean rawStdDevsPresent = rawStdDevs[6] != 0.0 || rawStdDevs[7] != 0.0;
    org.littletonrobotics.junction.Logger.recordOutput(
        "Vision/" + config.name() + "/rawStdDevsPresent", rawStdDevsPresent);
    org.littletonrobotics.junction.Logger.recordOutput(
        "Vision/" + config.name() + "/rawStdDevX", rawStdDevs[6]);
    org.littletonrobotics.junction.Logger.recordOutput(
        "Vision/" + config.name() + "/rawStdDevY", rawStdDevs[7]);
    inputs.stdDevs =
        calculateHybridStdDevs(rawStdDevs, mt2Estimate.avgTagDist, mt2Estimate.tagCount);

    int tid = (int) this.NT.getEntry("tid").getInteger(0);
    inputs.hubInSight = VisionConstants.hubTags.contains(tid);
    if (inputs.hubInSight) {
      inputs.primaryTid = tid;
      inputs.robotToHub = LimelightHelpers.getTargetPose3d_RobotSpace(config.name());
    }
  }

  private Matrix<N3, N1> calculateHybridStdDevs(
      double[] rawStdDevs, double avgTagDist, int numTags) {
    double baseX = rawStdDevs[6];
    double baseY = rawStdDevs[7];
    double baseYaw = rawStdDevs[11];

    double distanceScalar =
        1.0
            + Math.pow(
                avgTagDist / VisionConstants.maxReliableDistance,
                VisionConstants.distanceScalingExponent);

    double tagScalar = (numTags == 1) ? VisionConstants.singleTagPenalty : 1.0;
    double multiplier = VisionConstants.baseStddevMultiplier * distanceScalar * tagScalar;
    return VecBuilder.fill(baseX * multiplier, baseY * multiplier, baseYaw * multiplier);
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
