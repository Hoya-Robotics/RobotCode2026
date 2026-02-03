package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class LimelightLocalizationCamera implements LocalizationCameraIO {
  private final CameraConfig config;

  private final DoubleArraySubscriber cameraPoseTargetspaceSubscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final DoubleSubscriber clSubscriber;
  private final DoubleSubscriber tlSubscriber;
  private final IntegerSubscriber tidSubscriber;
  private final DoubleArraySubscriber rawFiducialSubscriber;
  private final DoubleArraySubscriber stdDevsSubscriber;

  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleArrayPublisher rewindCapturePublisher;
  private final DoublePublisher rewindEnablePublisher;
  private final IntegerPublisher throttlePublisher;
  private final IntegerPublisher imuModePublisher;

  private final Supplier<Rotation2d> externalYawSupplier;

  private boolean disabledState = false;
  private boolean rewindEnabled = VisionConstants.enableLimelightRewind;
  private double captures = 0.0;

  public LimelightLocalizationCamera(
      CameraConfig config, Supplier<Rotation2d> externalYawSupplier) {
    this.config = config;
    this.externalYawSupplier = externalYawSupplier;

    var table = NetworkTableInstance.getDefault().getTable(config.name());

    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    cameraPoseTargetspaceSubscriber =
        table.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});
    tlSubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    clSubscriber = table.getDoubleTopic("cl").subscribe(0.0);
    tidSubscriber = table.getIntegerTopic("tid").subscribe(0);
    rawFiducialSubscriber = table.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
    stdDevsSubscriber = table.getDoubleArrayTopic("stddevs").subscribe(new double[] {});

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    throttlePublisher = table.getIntegerTopic("throttle_set").publish();
    imuModePublisher = table.getIntegerTopic("imumode_set").publish();
    rewindCapturePublisher = table.getDoubleArrayTopic("capture_rewind").publish();
    rewindEnablePublisher = table.getDoubleTopic("rewind_enable_set").publish();

    rewindEnablePublisher.set(VisionConstants.enableLimelightRewind ? 1.0 : 0.0);
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void updateInputs(LocalizationInputs inputs) {
    NetworkTableInstance.getDefault().flush();

    double latency = tlSubscriber.get() + clSubscriber.get();

    orientationPublisher.set(
        new double[] {externalYawSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

    if (DriverStation.isDisabled()) {
      disabledState = true;
      throttlePublisher.set(200);
      imuModePublisher.set(1);
    } else if (disabledState) {
      disabledState = false;
      throttlePublisher.set(0);
      imuModePublisher.set(VisionConstants.matchImuMode);
    }

    int tid = (int) tidSubscriber.get();

    // Check for hub tags
    if (tid == 9 || tid == 10 || tid == 25 || tid == 26) {
      double[] cameraposeTargetspace = cameraPoseTargetspaceSubscriber.get();
      inputs.hubInView = true;

      var cameraToHub =
          new Transform3d(
              cameraposeTargetspace[0],
              cameraposeTargetspace[1],
              cameraposeTargetspace[2],
              new Rotation3d(
                  Units.degreesToRadians(cameraposeTargetspace[3]),
                  Units.degreesToRadians(cameraposeTargetspace[5]),
                  Units.degreesToRadians(cameraposeTargetspace[4])));
      inputs.hubObservation = new HubObservation(config.robotToCamera(), cameraToHub, tid);
    } else {
      inputs.hubInView = false;
    }

    // Read all updated megatag2 estimates
    List<MultitagPoseEstimate> observations = new ArrayList<>();
    var stddevQueue = stdDevsSubscriber.readQueue();
    int i = 0;
    for (var megatagSample : megatag2Subscriber.readQueue()) {
      Pose3d pose = deserializeLLPose(megatagSample.value);
      int tags = (int) megatagSample.value[7];
      double ambiguity = rawFiducialSubscriber.get()[6];
      MultitagPoseEstimate observation =
          new MultitagPoseEstimate(
              megatagSample.timestamp - megatagSample.value[6],
              pose,
              megatagSample.value[9],
              tags,
              tags > 1 ? 1.0 : 1.0 - ambiguity,
              stddevQueue[i].value);
      observations.add(observation);
      i += 1;
    }
    inputs.globalPoseObservations = observations.toArray(MultitagPoseEstimate[]::new);
  }

  @Override
  public void enableRewind(boolean enable) {
    this.rewindEnabled = enable;
    rewindEnablePublisher.set(enable ? 1.0 : 0.0);
  }

  @Override
  public void captureRewind(int duration) {
    rewindCapturePublisher.set(new double[] {captures, duration});
    captures += 1;
  }

  private static Pose3d deserializeLLPose(double[] values) {
    return new Pose3d(
        values[0],
        values[1],
        values[2],
        new Rotation3d(
            Units.degreesToRadians(values[3]),
            Units.degreesToRadians(values[4]),
            Units.degreesToRadians(values[5])));
  }
}
