package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] cameras;
  private final CameraConfig[] configs;
  private VisionIOInputsAutoLogged[] inputs;

  // Diagnostic counters
  private int acceptedMeasurements = 0;
  private int rejectedMeasurements = 0;
  private int hubObservations = 0;

  private Pose2d lastPoseEstimate = Pose2d.kZero;

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    this.inputs = new VisionIOInputsAutoLogged[cameras.length];
    this.configs = new CameraConfig[cameras.length];

    for (int i = 0; i < cameras.length; ++i) {
      this.configs[i] = cameras[i].getConfig();
      this.inputs[i] = new VisionIOInputsAutoLogged();
    }
    RobotState.getInstance().registerRewindCallback(this::captureRewind);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + configs[i].name(), inputs[i]);

      if (configs[i].filter().isPresent() && !configs[i].filter().get().getAsBoolean()) continue;

      var state = inputs[i];
      if (!state.isConnected) continue;

      // Global Pose Processing
      if (shouldAcceptMeasurement(state)) {
        Vector<N3> stdDevVec =
            VecBuilder.fill(
                state.stdDevs.get(0, 0), state.stdDevs.get(1, 0), state.stdDevs.get(2, 0));

        // Diagnostic logging for debugging vision pose estimation
        String camName = configs[i].name();
        Logger.recordOutput("Vision/" + camName + "/acceptedStdDevX", stdDevVec.get(0));
        Logger.recordOutput("Vision/" + camName + "/acceptedStdDevY", stdDevVec.get(1));
        Logger.recordOutput("Vision/" + camName + "/acceptedStdDevYaw", stdDevVec.get(2));
        Logger.recordOutput("Vision/" + camName + "/acceptedPose", state.poseEstimate);
        Logger.recordOutput("Vision/" + camName + "/acceptedTimestamp", state.timestamp);

        RobotState.getInstance()
            .addVisionMeasurement(
                new VisionObservation(
                    configs[i], state.poseEstimate, stdDevVec, Seconds.of(state.timestamp)));
        acceptedMeasurements++;
      } else if (state.numTags > 0) {
        rejectedMeasurements++;
      }
			lastPoseEstimate = state.poseEstimate;
    }

    // Log diagnostic counters
    Logger.recordOutput("Vision/acceptedMeasurements", acceptedMeasurements);
    Logger.recordOutput("Vision/rejectedMeasurements", rejectedMeasurements);
    Logger.recordOutput("Vision/hubObservations", hubObservations);
  }

  private boolean shouldAcceptMeasurement(VisionIOInputsAutoLogged state) {
    if (state.numTags == 0 || state.avgTagDist > VisionConstants.maxAcceptableDistance) {
      return false;
    }

    /*
    if (state.pose3d.getZ() > 0.2) {
      return false;
    }
    */

		/*
		if (lastPoseEstimate.getTranslation().getDistance(state.poseEstimate.getTranslation()) > VisionConstants.maxLatentDistance) {
			return false;
		}*/

    if (state.stdDevs != null) {
      double maxStd = Math.max(state.stdDevs.get(0, 0), state.stdDevs.get(1, 0));
      if (maxStd > VisionConstants.maxAcceptableStddev) {
        return false;
      }
    }

    return true;
  }

  public void captureRewind(double duration) {
    for (VisionIO camera : cameras) {
      camera.captureRewind(duration);
    }
  }
}
