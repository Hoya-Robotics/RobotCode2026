package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
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
      cameras[i].updateInputs(cameraInputs[i]);

      Logger.processInputs("Vision/" + cameras[i].getConfig().name(), cameraInputs[i]);

      for (int j = 0; j < cameraInputs[i].observations.length; ++j) {
        var obsv = cameraInputs[i].observations[j];
        if (obsv.isInvalid()) continue;

        double avgDist = obsv.avgTagDist();
        double tagCount = obsv.tagCount();
        double scale = (avgDist * avgDist) / tagCount;
        double linearStdDev = 0.03 * scale / tagCount;

        RobotState.getInstance()
            .addVisionMeasurement(
                new VisionObservation(
                    cameras[i].getConfig(),
                    obsv.pose().toPose2d(),
                    VecBuilder.fill(linearStdDev, linearStdDev, Float.POSITIVE_INFINITY),
                    Seconds.of(obsv.timestamp())));
      }
    }
  }

  public void captureRewind(double duration) {
    for (var camera : cameras) {
      camera.captureRewind(duration);
    }
  }
}
