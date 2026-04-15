package frc.robot.subsystems.vision;

import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonReal implements VisionIO {
  protected final CameraConfig config;
  protected final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhotonReal(CameraConfig config) {
    this.config = config;
    camera = new PhotonCamera(config.name());
    poseEstimator = new PhotonPoseEstimator(FieldConstants.aprilLayout, config.robotToCamera());
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    List<PoseObservation> observations = new ArrayList<>();
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      poseEstimator.resetHeadingData(
          results.get(0).getTimestampSeconds(),
          RobotState.getInstance().getEstimatedPose().getRotation());
    }

    for (var result : results) {
      if (!result.hasTargets()) continue;

      var estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
      if (estimatedPose.isEmpty())
        estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
      if (estimatedPose.isEmpty()) continue;

      var pose = estimatedPose.get();
      observations.add(
          new PoseObservation(
              pose.timestampSeconds,
              pose.estimatedPose,
              pose.targetsUsed.get(0).getPoseAmbiguity(),
              pose.targetsUsed.size(),
              pose.targetsUsed.stream()
                  .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                  .average()
                  .getAsDouble(),
              pose.targetsUsed.stream()
                  .mapToDouble(PhotonTrackedTarget::getArea)
                  .average()
                  .getAsDouble()));
    }

    inputs.connected = camera.isConnected();
    inputs.observations = observations.toArray(PoseObservation[]::new);
  }
}
