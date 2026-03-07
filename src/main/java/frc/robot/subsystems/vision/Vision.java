package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig.CameraConfig;
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
      this.configs[i] = cameras[i].getConfig();
      this.inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + configs[i].name(), inputs[i]);

      var state = inputs[i];
      if (!state.isConnected) continue;

      // TODO: Process vision data, calculate stdDevs, upload measurement
    }
  }
}
