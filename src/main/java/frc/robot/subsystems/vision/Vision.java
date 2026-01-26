package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import frc.robot.FieldConstants;
import frc.robot.RobotState.*;
import frc.robot.StateSubsystem;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class Vision extends StateSubsystem<frc.robot.subsystems.vision.Vision.SystemState> {
  public enum SystemState {
    BEST_TAG_2D,
    ALL_TAGS_2D,
  }

  private VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] inputs;
  private final Consumer<Angle> setYaw;

  public Vision(VisionIO[] cameras, Consumer<Angle> setYaw) {
    this.setYaw = setYaw;
    this.cameras = cameras;
    this.inputs = new VisionIOInputsAutoLogged[cameras.length];
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; ++i) {
      cameras[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/camera" + i, inputs[i]);
    }

    applyState();
  }

  @Override
  protected void applyState() {
    for (int i = 0; i < cameras.length; ++i) {
      final VisionIOInputsAutoLogged input = inputs[i];
      // setYaw.accept(input.megatagYaw);

      switch (getCurrentState()) {
        case BEST_TAG_2D -> {
          final Pose2d estimate = poseEstimate2d(input.bestTarget);
          /*
          RobotState.getInstance()
              .addVisionObservation(
                  new AprilTagObservation(input.bestTarget.timestamp(), estimate));
                  */
        }
        case ALL_TAGS_2D -> {
          for (var tag : input.allTargets) {
            final Pose2d estimate = poseEstimate2d(tag);
            /*
            RobotState.getInstance()
                .addVisionObservation(new AprilTagObservation(tag.timestamp(), estimate));
                */
          }
        }
      }
    }
  }

  /*
   * Estimates robot pose relative to april tag using only 2 DOF (yaw and distance)
   * Simpler and more robust to noise during motion than 6 DOF methods like Megatag
   */
  public Pose2d poseEstimate2d(VisionIO.AprilFiducial tag) {
    final Pose3d tagPose = FieldConstants.aprilLayout.getTagPose(tag.tid()).get();
    final Transform3d robotToCamera = VisionConstants.robotToCameras[tag.camera()];
    final Rotation3d camRot = robotToCamera.getRotation();

    // ground distance = tag height / tan(camera pitch + ty)
    final double distance = tagPose.getZ() / Math.tan(camRot.getY() + Math.toRadians(tag.ty()));

    // Project camera tx to ground tx: tan(ground tx) = tan(cam tx) / cos(cam yaw)
    final double groundTx = Math.atan(Math.tan(Math.toRadians(tag.tx())) / Math.cos(camRot.getZ()));

    final Transform2d tagToCamera =
        new Transform2d(
            new Translation2d(distance, Rotation2d.fromRadians(-groundTx)), Rotation2d.kZero);

    final Pose2d cameraPose = tagPose.toPose2d().transformBy(tagToCamera);
    return cameraPose.transformBy(VisionConstants.cameraToRobot2d[tag.camera()]);
  }
}
