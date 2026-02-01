package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.CameraConfig;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private static VisionSystemSim simWorld;
  private final PhotonCameraSim cameraSim;
  private final PhotonCamera camera;
  private final int index;
  private final Supplier<Pose2d> poseSupplier;
  private final CameraConfig config;

  public VisionIOSim(int index, Supplier<Pose2d> poseSupplier) {
    this.index = index;
    this.config = RobotConfig.cameras.get(index);
    this.poseSupplier = poseSupplier;

    if (simWorld == null) {
      simWorld = new VisionSystemSim("main");
      simWorld.addAprilTags(FieldConstants.aprilLayout);
    }
    camera = new PhotonCamera(config.name());
    var camProps = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, camProps);

    simWorld.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    simWorld.update(poseSupplier.get());

    for (var result : camera.getAllUnreadResults()) {
      var best = result.getBestTarget();
      var transform = best.getBestCameraToTarget();
    }
  }
}
