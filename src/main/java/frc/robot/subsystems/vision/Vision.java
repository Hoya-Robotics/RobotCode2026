package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.subsystems.vision.LocalizationCameraIO.MultitagPoseEstimate;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final LocalizationCameraIO[] localizers;
  private LocalizationInputsAutoLogged[] localizerInputs;

  public Vision(LocalizationCameraIO[] localizers) {
    this.localizers = localizers;
    this.localizerInputs = new LocalizationInputsAutoLogged[localizers.length];
    for (int i = 0; i < localizers.length; ++i) {
      localizerInputs[i] = new LocalizationInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < localizers.length; ++i) {
      localizers[i].updateInputs(localizerInputs[i]);
      var config = localizers[i].getConfig();
      Logger.processInputs("Vision/" + config.name(), localizerInputs[i]);

      var inputs = localizerInputs[i];
      if (inputs.hubInView) {
        RobotState.getInstance().addHubObservation(inputs.hubObservation);
      }

      for (var multiEst : inputs.globalPoseObservations) {
        tryProcessMultitagEstimate(multiEst)
            .ifPresent(
                (visionEstimate) -> {
                  RobotState.getInstance().addVisionObservation(visionEstimate);
                });
      }
    }
  }

  private Optional<VisionFieldPoseEstimate> tryProcessMultitagEstimate(
      MultitagPoseEstimate poseEstimate) {
    if (poseEstimate.avgTagDistance() > VisionConstants.multitagTagDistanceThreshold.in(Meters)) {
      return Optional.empty();
    }

    double qualityFactor = 1.0 / poseEstimate.quality();
    double xStd = poseEstimate.stdDevs()[0] * qualityFactor;
    double yStd = poseEstimate.stdDevs()[1] * qualityFactor;
    double omegaStd = poseEstimate.stdDevs()[2] * qualityFactor;
    double xyStd = Math.max(xStd, yStd);

    return Optional.of(
        new VisionFieldPoseEstimate(
            poseEstimate.timestamp(),
            poseEstimate.pose().toPose2d(),
            VecBuilder.fill(xyStd, xyStd, omegaStd)));
  }
}
