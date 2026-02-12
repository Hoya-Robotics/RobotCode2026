package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonSim implements VisionIO {
  private static VisionSystemSim visionSim;

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final CameraConfig config;
  private final PhotonPoseEstimator poseEstimator;
  private static final Vector<N3> kdefaultStdDevs = VecBuilder.fill(0.3, 0.3, 0.3);

  public VisionIOPhotonSim(CameraConfig config) {
    this.config = config;
    camera = new PhotonCamera(config.name());
    cameraSim = new PhotonCameraSim(camera, config.simProps(), FieldConstants.aprilLayout);

    cameraSim.setMaxSightRange(5.0);
    cameraSim.enableProcessedStream(false);
    cameraSim.enableRawStream(false);
    cameraSim.enableDrawWireframe(false);

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilLayout);
    }
    visionSim.addCamera(cameraSim, config.robotToCamera());
    poseEstimator = new PhotonPoseEstimator(FieldConstants.aprilLayout, config.robotToCamera());
  }

  @Override
  public CameraConfig getConfig() {
    return config;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(RobotState.getInstance().getEstimatedPose());
    for (var result : camera.getAllUnreadResults()) {
      var bestEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
      if (bestEstimate.isEmpty()) {
        bestEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      bestEstimate.ifPresent(
          estimate -> {
            double avgDist = 0.0;
            int tags = 0;
            for (var target : result.getTargets()) {
              var tagPose = poseEstimator.getFieldTags().getTagPose(target.fiducialId);
              if (tagPose.isEmpty()) continue;
              avgDist +=
                  estimate
                      .estimatedPose
                      .toPose2d()
                      .getTranslation()
                      .getDistance(tagPose.get().toPose2d().getTranslation());
              tags += 1;
            }

            inputs.poseEstimate =
                new MultitagPoseEstimate(
                    estimate.estimatedPose.toPose2d(),
                    avgDist / tags,
                    tags,
                    kdefaultStdDevs,
                    estimate.timestampSeconds);
          });
    }
  }
}
