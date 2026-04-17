package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.util.LimelightHelpers;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelightReal implements VisionIO {
  private final CameraConfig config;
  private final DoubleSubscriber latencySubscriber;
  private final IntegerPublisher throttlePublisher;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber hardwareMetricSubcriber;
  private int lastThrottleValue = 0;

  public VisionIOLimelightReal(CameraConfig config) {
    this.config = config;

    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(config.name());
    latencySubscriber = networkTable.getDoubleTopic("tl").subscribe(0.0);
    megatag1Subscriber =
        networkTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    throttlePublisher = networkTable.getIntegerTopic("throttle_set").publish();
    hardwareMetricSubcriber = networkTable.getDoubleArrayTopic("hw").subscribe(new double[] {});

    LimelightHelpers.setCameraPose_RobotSpace(
        config.name(),
        config.robotToCamera().getTranslation().getX(),
        config.robotToCamera().getTranslation().getY(),
        config.robotToCamera().getTranslation().getZ(),
        config.robotToCamera().getRotation().getMeasureX().in(Degrees),
        config.robotToCamera().getRotation().getMeasureY().in(Degrees),
        config.robotToCamera().getRotation().getMeasureZ().in(Degrees));
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
    double[] hardwareMetrics = hardwareMetricSubcriber.get();
    if (hardwareMetrics.length >= 1) {
      Logger.recordOutput("Vision/" + config.name() + "/cpuTemp", hardwareMetrics[0]);
    }

    if (DriverStation.isDisabled() && lastThrottleValue == 0) {
      lastThrottleValue = 200;
      throttlePublisher.set(200);
    } else if (DriverStation.isEnabled() && lastThrottleValue != 0) {
      lastThrottleValue = 0;
      throttlePublisher.set(0);
    }

    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    Set<Integer> tagSet = new HashSet<>();
    List<PoseObservation> observations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 0; i < rawSample.value.length; i += 7) {
        tagSet.add((int) rawSample.value[i]);
      }

      Pose3d pose =
          new Pose3d(
              rawSample.value[0],
              rawSample.value[1],
              rawSample.value[2],
              new Rotation3d(
                  Units.degreesToRadians(rawSample.value[3]),
                  Units.degreesToRadians(rawSample.value[4]),
                  Units.degreesToRadians(rawSample.value[5])));

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
    inputs.tagIds = Arrays.stream(tagSet.toArray(Integer[]::new)).mapToInt(i -> i).toArray();
  }
}
