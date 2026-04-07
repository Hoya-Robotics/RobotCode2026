package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import java.util.LinkedList;
import java.util.List;
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
    List<Pose3d> acceptedPoseEstimates = new LinkedList<>();
    List<Pose3d> rejectedPoseEstimates = new LinkedList<>();
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(cameraInputs[i]);

      Logger.processInputs("Vision/" + cameras[i].getConfig().name(), cameraInputs[i]);

      if (cameraInputs[i].observations == null) continue;
      for (int j = 0; j < cameraInputs[i].observations.length; ++j) {
        var obsv = cameraInputs[i].observations[j];
        if (obsv.isInvalid()) {
          rejectedPoseEstimates.add(obsv.pose());
          continue;
        }
        acceptedPoseEstimates.add(obsv.pose());

        double avgDist = obsv.avgTagDist();
        double tagCount = (double) obsv.tagCount();
        double scale = (avgDist * avgDist) / tagCount;
        double xDev =
            (cameraInputs[i].stddevs[0] * scale / tagCount)
                * (1.0 / VisionConstants.linearTrustFactor);
        double yDev =
            (cameraInputs[i].stddevs[1] * scale / tagCount)
                * (1.0 / VisionConstants.linearTrustFactor);

        RobotState.getInstance()
            .addVisionMeasurement(
                new VisionObservation(
                    cameras[i].getConfig(),
                    obsv.pose().toPose2d(),
                    VecBuilder.fill(xDev, yDev, Double.MAX_VALUE),
                    Seconds.of(obsv.timestamp())));
      }
    }

    Logger.recordOutput(
        "Vision/Summary/acceptedPoses", acceptedPoseEstimates.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/rejectedPoses", rejectedPoseEstimates.toArray(Pose3d[]::new));
  }

  public void captureRewind(double duration) {
    for (var camera : cameras) {
      camera.captureRewind(duration);
    }
  }
}
