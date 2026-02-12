package frc.robot.subsystems.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.subsystems.vision.VisionIO.MultitagPoseEstimate;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] cameras;
  private final CameraConfig[] configs;
  private VisionIOInputsAutoLogged[] inputs;

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    this.inputs = new VisionIOInputsAutoLogged[cameras.length];
    this.configs = new CameraConfig[cameras.length];
    for (int i = 0; i < cameras.length; ++i) {
      inputs[i] = new VisionIOInputsAutoLogged();
      configs[i] = cameras[i].getConfig();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + configs[i].name(), inputs[i]);
    }

    for (var input : inputs) {
      if (input.poseEstimate == null) continue;
      estimateStdDevs(input.poseEstimate)
          .ifPresent(
              stdDevs -> {
                RobotState.getInstance()
                    .addVisionMeasurement(
                        new VisionObservation(
                            input.poseEstimate.pose(), stdDevs, input.poseEstimate.timestamp()));
              });
    }
  }

  // TODO: implement
  private static Optional<Vector<N3>> estimateStdDevs(MultitagPoseEstimate estimate) {
    if (estimate.tags() == 0) return Optional.empty();
    return Optional.of(estimate.stdDevs());
  }
}
