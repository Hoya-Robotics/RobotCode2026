package frc.robot.subsystems.vision;

import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonSim extends VisionIOPhotonReal {
  private static VisionSystemSim visionSim;
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonSim(CameraConfig config) {
    super(config);
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilLayout);
    }

    cameraSim = new PhotonCameraSim(camera, config.simProps(), FieldConstants.aprilLayout);
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(RobotState.getInstance().getSimulatedPose());
    super.updateInputs(inputs);
  }
}
