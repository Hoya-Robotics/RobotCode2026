package frc.robot.subsystems.vision;

import frc.robot.FieldConstants;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonSimLocalizationCamera implements LocalizationCameraIO {
  private static VisionSystemSim visionWorld;
  private final CameraConfig config;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator poseEstimator;

  public PhotonSimLocalizationCamera(CameraConfig config) {
    this.config = config;

    if (visionWorld == null) {
      visionWorld = new VisionSystemSim("main");
      visionWorld.addAprilTags(FieldConstants.aprilLayout);
    }

    camera = new PhotonCamera(config.name());
    cameraSim =
        new PhotonCameraSim(camera, SimConstants.LL4CameraProperties, FieldConstants.aprilLayout);
    poseEstimator = new PhotonPoseEstimator(FieldConstants.aprilLayout, config.robotToCamera());

    visionWorld.addCamera(cameraSim, config.robotToCamera());
    cameraSim.enableDrawWireframe(true);
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void updateInputs(LocalizationInputs inputs) {
    visionWorld.update(RobotState.getInstance().getSimulatedDrivePose());

    var results = camera.getAllUnreadResults();
    List<MultitagPoseEstimate> observations = new ArrayList<>();
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

      var multitagEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (multitagEst.isEmpty()) {
        multitagEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (multitagEst.isEmpty()) {
        continue;
      }

      observations.add(
          new MultitagPoseEstimate(
              multitagEst.get().timestampSeconds,
              multitagEst.get().estimatedPose,
              totalTagDistance / multitagEst.get().targetsUsed.size(),
              multitagEst.get().targetsUsed.size(),
              1.0,
              new double[] {0.03, 0.03, 0.03}));
    }
    inputs.hubInView = hubInView;
    inputs.globalPoseObservations = observations.toArray(MultitagPoseEstimate[]::new);
  }
}
