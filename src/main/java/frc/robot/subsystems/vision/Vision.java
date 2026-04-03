package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] cameraInputs;

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    this.cameraInputs = new VisionIOInputsAutoLogged[cameras.length];
    for (int i = 0; i < cameras.length; ++i) {
      cameraInputs[i] = new VisionIOInputsAutoLogged();
    }
    RobotState.getInstance().registerVision(this);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      // cameras[i].setRobotOrientation(RobotState.getInstance().getEstimatedPose().getRotation());
      cameras[i].updateInputs(cameraInputs[i]);

      Logger.processInputs("Vision/" + cameras[i].getConfig().name(), cameraInputs[i]);

      processCamera(cameras[i].getConfig(), cameraInputs[i])
          .ifPresent(obsv -> RobotState.getInstance().addVisionMeasurement(obsv));
    }
  }

  private static Optional<VisionObservation> processCamera(
      CameraConfig config, VisionIOInputs inputs) {
    if (!isTrustableMeasurement(inputs)) return Optional.empty();

    /*
      double scale = 1.0 / inputs.mt1.quality();

      double xStd = inputs.stddevs[0] * scale * 0.7;
      double yStd = inputs.stddevs[1] * scale * 0.7;
    */
    double tagCount = inputs.mt1.tagCount();
    double avgDist = inputs.mt1.avgTagDist();
    double scale = (avgDist * avgDist) / tagCount;
    double linearStdDev = 0.03 * scale / tagCount;

    return Optional.of(
        new VisionObservation(
            config,
            inputs.mt1.pose(),
            VecBuilder.fill(linearStdDev, linearStdDev, Float.POSITIVE_INFINITY),
            Seconds.of(inputs.mt1.timestamp())));
  }

  private static boolean isTrustableMeasurement(VisionIOInputs inputs) {
    if (inputs.mt1 == null) return false;

    var estimate = inputs.mt1;
    Pose2d pose = estimate.pose();

    boolean tagless = estimate.tagCount() == 0;
    boolean tooHigh =
        inputs.pose3d != null && Math.abs(inputs.pose3d.getZ()) > VisionConstants.zThreshold;
    boolean outofbounds =
        pose.getX() < 0.0
            || pose.getMeasureX().gt(FieldConstants.fieldLength)
            || pose.getY() < 0.0
            || pose.getMeasureY().gt(FieldConstants.fieldWidth);

    if (tagless || tooHigh || outofbounds) {
      return false;
    }

    if (estimate.tagCount() < 2) {
      boolean ambiguous =
          Arrays.stream(inputs.fiducials)
              .anyMatch(f -> f.ambiguity() > VisionConstants.maxSingleTagAmbiguity);
      boolean smallArea = estimate.avgTagArea() < VisionConstants.minSingleTagArea;

      if (ambiguous || smallArea) return false;
    }

    return true;
  }

  public void setRobotToCamera(String cameraName, Transform3d robotToCamera) {
    Arrays.stream(cameras)
        .filter(c -> c.getConfig().name().equals(cameraName))
        .findAny()
        .ifPresent(c -> c.setRobotToCamera(robotToCamera));
  }

  public void captureRewind(double duration) {
    for (var camera : cameras) {
      camera.captureRewind(duration);
    }
  }
}
