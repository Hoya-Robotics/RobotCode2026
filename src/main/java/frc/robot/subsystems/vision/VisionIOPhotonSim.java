package frc.robot.subsystems.vision;

import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonSim implements VisionIO {
  private static VisionSystemSim visionSim;

  private final CameraConfig config;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonSim(CameraConfig config) {
    this.config = config;
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilLayout);
    }
    this.camera = new PhotonCamera(config.name());
    this.cameraSim = new PhotonCameraSim(camera); // TODO: simulate ll4 properties
    visionSim.addCamera(cameraSim, config.robotToCamera()); // TODO: update turret cam pose
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {}

  @Override
  public CameraConfig getConfig() {
    return config;
  }
}
