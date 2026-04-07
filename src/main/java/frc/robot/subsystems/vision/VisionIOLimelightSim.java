package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOLimelightSim extends VisionIOLimelight {
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final PhotonPoseEstimator estimator;

  public VisionIOLimelightSim(CameraConfig config) {
    super(config);

    estimator = new PhotonPoseEstimator(FieldConstants.aprilLayout, config.robotToCamera());
    camera = new PhotonCamera(config.name());
    cameraSim = new PhotonCameraSim(camera, config.simProps());
    RobotState.getInstance().getVisionSim().addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    RobotState.getInstance().getVisionSim().update(RobotState.getInstance().getSimulatedPose());

    var table = NetworkTableInstance.getDefault().getTable(getConfig().name());
    boolean seesTarget = false;
    for (var result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;
      var estimate = estimator.estimateCoprocMultiTagPose(result);
      if (estimate.isEmpty()) {
        estimate = estimator.estimateLowestAmbiguityPose(result);
      }
      if (estimate.isEmpty()) continue;
      table
          .getEntry("botpose_wpiblue")
          .setDoubleArray(
              estimateToBotposeArray(estimate.get(), result).stream()
                  .mapToDouble(Double::doubleValue)
                  .toArray());
      table
          .getEntry("rawfiducials")
          .setDoubleArray(
              estimateToRawFiducialArray(estimate.get(), result).stream()
                  .mapToDouble(Double::doubleValue)
                  .toArray());
      table
          .getEntry("stddevs")
          .setDoubleArray(
              new double[] {
                VisionConstants.defaultLinearStddevPhoton, VisionConstants.defaultLinearStddevPhoton
              });
      table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
      seesTarget = true;
    }
    table.getEntry("tv").setInteger(seesTarget ? 1 : 0);

    super.updateInputs(inputs);
  }

  private static List<Double> estimateToBotposeArray(
      EstimatedRobotPose poseEstimate, PhotonPipelineResult result) {
    Pose3d pose = poseEstimate.estimatedPose;
    return List.of(
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        0.0,
        0.0,
        pose.getRotation().getMeasureZ().in(Units.Degrees),
        result.metadata.getLatencyMillis(),
        (double) poseEstimate.targetsUsed.size(),
        0.0,
        poseEstimate.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .getAsDouble(),
        result.getBestTarget().getArea());
  }

  private static List<Double> estimateToRawFiducialArray(
      EstimatedRobotPose poseEstimate, PhotonPipelineResult result) {
    List<Double> data = new ArrayList<>();
    for (var target : poseEstimate.targetsUsed) {
      data.addAll(
          List.of(
              (double) target.fiducialId,
              target.getYaw(),
              target.getPitch(),
              target.area,
              0.0,
              0.0,
              target.getPoseAmbiguity()));
    }
    return data;
  }
}
