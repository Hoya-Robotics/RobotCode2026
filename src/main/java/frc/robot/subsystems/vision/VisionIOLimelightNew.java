package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.util.LimelightHelpers;
import java.util.LinkedList;
import java.util.List;

public class VisionIOLimelightNew implements VisionIO {
  private final CameraConfig config;
  private final DoubleSubscriber latencySubscriber;
  private final IntegerPublisher throttlePublisher;
  private final DoubleArraySubscriber megatag1Subscriber;
  private int lastThrottleValue = 0;

  public VisionIOLimelightNew(CameraConfig config) {
    this.config = config;

    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(config.name());
    latencySubscriber = networkTable.getDoubleTopic("tl").subscribe(0.0);
    megatag1Subscriber =
        networkTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    throttlePublisher = networkTable.getIntegerTopic("throttle_set").publish();

    LimelightHelpers.setRewindEnabled(config.name(), true);
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void captureRewind(double duration) {
    LimelightHelpers.triggerRewindCapture(config.name(), duration);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (DriverStation.isDisabled() && lastThrottleValue == 0) {
      lastThrottleValue = 200;
      throttlePublisher.set(200);
    } else if (DriverStation.isEnabled() && lastThrottleValue != 0) {
      lastThrottleValue = 0;
      throttlePublisher.set(0);
    }

    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    List<PoseObservation> observations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      Pose3d pose =
          new Pose3d(
              rawSample.value[0],
              rawSample.value[1],
              rawSample.value[2],
              new Rotation3d(rawSample.value[3], rawSample.value[4], rawSample.value[5]));

      observations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              pose,
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              rawSample.value[10]));
    }

    inputs.observations = observations.toArray(PoseObservation[]::new);
  }
}
