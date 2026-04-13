package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private final CameraConfig config;
  private final NetworkTable NT;
  private boolean lastDisabled = true;

  private final DoubleArraySubscriber mt1Subscriber;
  private final DoubleArraySubscriber stddevSubscriber;
  private final DoubleArraySubscriber rawFiducialSubscriber;

  public VisionIOLimelight(CameraConfig config) {
    this.config = config;
    this.NT = NetworkTableInstance.getDefault().getTable(config.name());
    mt1Subscriber = NT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    stddevSubscriber = NT.getDoubleArrayTopic("stddevs").subscribe(new double[] {});
    rawFiducialSubscriber = NT.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});

    LimelightHelpers.setRewindEnabled(config.name(), true);
    LimelightHelpers.setCameraPose_RobotSpace(
        config.name(),
        config.robotToCamera().getX(),
        config.robotToCamera().getY(),
        config.robotToCamera().getZ(),
        config.robotToCamera().getRotation().getMeasureX().in(Degrees),
        config.robotToCamera().getRotation().getMeasureY().in(Degrees),
        config.robotToCamera().getRotation().getMeasureZ().in(Degrees));
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

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
          rawStddevs[0] + 1e-6, // MT1_x
          rawStddevs[1] + 1e-6 // MT1_y
        };

    var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name());
    Pose3d pose3d = LimelightHelpers.getBotPose3d_wpiBlue(config.name());
    LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(config.name());
    inputs.observations =
        new PoseObservation[] {
          new PoseObservation(
              mt1.timestampSeconds,
              pose3d,
              rawFiducials.length == 0 ? 0.0 : rawFiducials[0].ambiguity,
              mt1.tagCount,
              mt1.avgTagDist,
              mt1.avgTagArea)
        };
  }

  @Override
  public void captureRewind(double duration) {
    LimelightHelpers.triggerRewindCapture(config.name(), duration);
  }
}
