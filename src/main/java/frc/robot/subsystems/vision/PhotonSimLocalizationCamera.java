package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonSimLocalizationCamera implements LocalizationCameraIO {
  private static VisionSystemSim visionWorld;
  private final CameraConfig config;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;

  public PhotonSimLocalizationCamera(CameraConfig config) {
    this.config = config;

    if (visionWorld == null) {
      visionWorld = new VisionSystemSim("main");
      visionWorld.addAprilTags(FieldConstants.aprilLayout);
    }

    SimCameraProperties props = new SimCameraProperties();
    camera = new PhotonCamera(config.name());
    cameraSim = new PhotonCameraSim(camera, props);

    visionWorld.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void updateInputs(LocalizationInputs inputs) {
    visionWorld.update(RobotState.getInstance().getSimulatedDrivePose());

    var results = camera.getAllUnreadResults();
    List<MT2Observation> observations = new ArrayList<>();
    // TODO: filter multiple hub results
    // TODO: handle single tag pose estimation
    boolean hubInView = false;
    for (var result : results) {
      double totalTagDistance = 0.0;
      for (var target : result.getTargets()) {
        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();

        int tid = target.fiducialId;
        // Hub target
        if (tid == 9 || tid == 10 || tid == 25 || tid == 26) {
          hubInView = true;
          inputs.hubObservation =
              new HubObservation(config.robotToCamera(), target.bestCameraToTarget, tid);
          break;
        }
      }
      final double avgTagDistance = totalTagDistance / result.targets.size();

      result.multitagResult.ifPresent(
          (pnpResult) -> {
            Transform3d cameraToField = pnpResult.estimatedPose.best;
            Transform3d robotToField = cameraToField.plus(config.robotToCamera());
            Pose3d robotPose =
                new Pose3d(robotToField.getTranslation(), robotToField.getRotation());

            observations.add(
                new MT2Observation(
                    result.getTimestampSeconds(),
                    robotPose,
                    avgTagDistance,
                    pnpResult.fiducialIDsUsed.size()));
          });
    }
    inputs.hubInView = hubInView;
  }
}
