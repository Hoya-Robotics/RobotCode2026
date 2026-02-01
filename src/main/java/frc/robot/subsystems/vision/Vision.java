package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState.*;

public class Vision extends SubsystemBase {
  public Vision() {}
}

/*  public Pose2d poseEstimate2d(VisionIO.AprilFiducial tag) {
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
*/
