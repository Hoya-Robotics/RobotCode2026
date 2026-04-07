package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.util.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;

public class VisionIOLimelight implements VisionIO {
  private final CameraConfig config;
  private final NetworkTable NT;
  private boolean lastDisabled = true;

  private final DoubleArraySubscriber mt1Subscriber;
  private final DoubleArraySubscriber rawFiducialsSubscriber;
  private final DoubleArraySubscriber stddevSubscriber;

  public VisionIOLimelight(CameraConfig config) {
    this.config = config;
    this.NT = NetworkTableInstance.getDefault().getTable(config.name());
    mt1Subscriber = NT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    rawFiducialsSubscriber = NT.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
    stddevSubscriber = NT.getDoubleArrayTopic("stddevs").subscribe(new double[] {});

    LimelightHelpers.setRewindEnabled(config.name(), true);
    setRobotToCamera(config.robotToCamera());
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void setRobotToCamera(Transform3d robotToCamera) {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    boolean disabled = DriverStation.isDisabled();
    if (disabled != lastDisabled) {
      if (disabled) {
        LimelightHelpers.SetIMUMode(config.name(), 1);
        LimelightHelpers.SetThrottle(config.name(), 1000);
      } else {
        LimelightHelpers.SetIMUMode(config.name(), 3);
        LimelightHelpers.SetThrottle(config.name(), 0);
      }
      lastDisabled = disabled;
    }

    inputs.seesTarget = NT.getEntry("tv").getDouble(0.0) == 1.0;
    if (!inputs.seesTarget) return;

    double[] rawStddevs = stddevSubscriber.get();
    inputs.stddevs =
        new double[] {
          rawStddevs[0], // MT1_x
          rawStddevs[1] // MT1_y
        };

    var mt1Stream = mt1Subscriber.readQueue();
    var fiducialsStream = rawFiducialsSubscriber.readQueueValues();
    List<PoseObservation> observations = new ArrayList<>();
    for (int i = 0; i < mt1Stream.length; ++i) {
      double[] sample = mt1Stream[i].value;
      Pose3d pose =
          new Pose3d(
              sample[0], sample[1], sample[2], new Rotation3d(sample[3], sample[4], sample[5]));

      observations.add(
          new PoseObservation(
              mt1Stream[i].timestamp * 1.0e-6 - sample[6] * 1.0e-3,
              pose,
              fiducialsStream.length > i ? fiducialsStream[i][0] : 0.0,
              (int) sample[7],
              sample[9],
              sample[10]));
    }
    inputs.observations = observations.toArray(PoseObservation[]::new);
  }

  @Override
  public void captureRewind(double duration) {
    LimelightHelpers.triggerRewindCapture(config.name(), duration);
  }
}
