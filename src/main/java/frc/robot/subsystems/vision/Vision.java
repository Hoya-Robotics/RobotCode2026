package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] cameraInputs;
  private List<Pose3d> acceptedPoseEstimates = new LinkedList<>();
  private List<Pose3d> rejectedPoseEstimates = new LinkedList<>();

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

      if (!cameraInputs[i].connected || cameraInputs[i].observations == null) continue;
      for (int j = 0; j < cameraInputs[i].observations.length; ++j) {
        var obsv = cameraInputs[i].observations[j];
        if (obsv.isInvalid()) {
          rejectedPoseEstimates.add(obsv.pose());
          continue;
        }
        acceptedPoseEstimates.add(obsv.pose());

        Logger.recordOutput(
            "Vision/" + cameras[i].getConfig().name() + "/visionTargets",
            Arrays.stream(cameraInputs[i].tagIds)
                .mapToObj(id -> FieldConstants.aprilLayout.getTagPose(id))
                .filter(Optional::isPresent)
                .map(Optional::get)
                .toArray(Pose3d[]::new));

        double avgDist = obsv.avgTagDist();
        double tagCount = (double) obsv.tagCount();
        double scale = (avgDist * avgDist) / tagCount;
        double xDev =
            (VisionConstants.defaultLinearStddevPhoton * scale / tagCount)
                * (1.0 / VisionConstants.linearTrustFactor);
        double yDev =
            (VisionConstants.defaultLinearStddevPhoton * scale / tagCount)
                * (1.0 / VisionConstants.linearTrustFactor);

        RobotState.getInstance()
            .addVisionMeasurement(
                new VisionObservation(
                    cameras[i].getConfig(),
                    obsv.pose().toPose2d(),
                    VecBuilder.fill(xDev, yDev, 9999.0),
                    Seconds.of(obsv.timestamp())));
      }
    }

    Logger.recordOutput(
        "Vision/Summary/acceptedPoses", acceptedPoseEstimates.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/rejectedPoses", rejectedPoseEstimates.toArray(Pose3d[]::new));

    acceptedPoseEstimates.clear();
    rejectedPoseEstimates.clear();
  }

  public void captureRewind(double duration) {
    for (var camera : cameras) {
      camera.captureRewind(duration);
    }
  }
}
