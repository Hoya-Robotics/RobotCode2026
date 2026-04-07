package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOLimelightSim extends VisionIOLimelight {
  private static VisionSystemSim visionSim;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator estimator;

  public VisionIOLimelightSim(CameraConfig config) {
    super(config);

    estimator = new PhotonPoseEstimator(FieldConstants.aprilLayout, config.robotToCamera());
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilLayout);
    }
    camera = new PhotonCamera(config.name());
    cameraSim = new PhotonCameraSim(camera, config.simProps());
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(RobotState.getInstance().getSimulatedPose());

    var table = NetworkTableInstance.getDefault().getTable(getConfig().name());
    boolean seesTarget = false;
    for (var result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;
      var estimate = estimator.estimateCoprocMultiTagPose(result);
      if (estimate.isEmpty()) {
        estimate = estimator.estimateLowestAmbiguityPose(result);
      }
      if (estimate.isEmpty()) continue;
      List<Double> ntData = estimateToLLArray(estimate.get(), result);
      table
          .getEntry("botpose_wpiblue")
          .setDoubleArray(ntData.stream().mapToDouble(Double::doubleValue).toArray());
      table
          .getEntry("botpose_orb_wpiblue")
          .setDoubleArray(ntData.stream().mapToDouble(Double::doubleValue).toArray());
      table
          .getEntry("stddevs")
          .setDoubleArray(
              new double[] {0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
      seesTarget = true;
    }
    table.getEntry("tv").setInteger(seesTarget ? 1 : 0);

    super.updateInputs(inputs);
  }

  /*
   * Translates photonvision pose estimate and result data
   * to limelight double array for network table injection
   */
  private static List<Double> estimateToLLArray(
      EstimatedRobotPose poseEstimate, PhotonPipelineResult result) {
    Pose3d pose = poseEstimate.estimatedPose;

    List<Double> data =
        new ArrayList<>(
            List.of(
                pose.getX(),
                pose.getY(),
                pose.getZ(),
                0.0,
                0.0,
                pose.getRotation().getMeasureZ().in(Units.Degrees),
                result.metadata.getLatencyMillis(),
                (double) poseEstimate.targetsUsed.size(),
                0.0,
                0.0,
                result.getBestTarget().getArea()));
    for (var fiducial : result.getTargets()) {
      data.addAll(
          List.of(
              (double) fiducial.getFiducialId(),
              fiducial.getYaw(),
              fiducial.getPitch(),
              fiducial.area,
              0.0,
              0.0,
              fiducial.getPoseAmbiguity()));
    }
    return data;
  }
}
