package frc.robot.subsystems.vision;

import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState;
import org.photonvision.simulation.PhotonCameraSim;

public class VisionIOPhotonSim extends VisionIOPhotonReal {
  private final PhotonCameraSim cameraSim;

  public VisionIOPhotonSim(CameraConfig config) {
    super(config);

    cameraSim = new PhotonCameraSim(camera, config.simProps(), FieldConstants.aprilLayout);
    RobotState.getInstance().getVisionSim().addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    RobotState.getInstance().getVisionSim().update(RobotState.getInstance().getSimulatedPose());
    super.updateInputs(inputs);
  }
}
