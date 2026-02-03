package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConfig;
import frc.robot.RobotState;
import frc.robot.RobotConfig.*;
import org.littletonrobotics.junction.Logger;

public class VisionProto {
  /*
  private static CameraConfig intakeCamera = new CameraConfig(
    "intake",
    new Transform3d(),
    CameraType.FUEL_DETECT
  );*/
	public static void logCameras() {
		var robot = new Pose3d(RobotState.getInstance().getOdometryPose());
		for (var cam : RobotConfig.cameras) {
			Logger.recordOutput("VisionProto/" + cam.name(), robot.transformBy(cam.robotToCamera()));
		}
	}

  public static void wideFOVFrontSetup() {
    var turretLeft =
        new CameraConfig(
            "turretLeft",
            new Transform3d(
                Units.inchesToMeters(-6.194),
                Units.inchesToMeters(13.0),
                Units.inchesToMeters(20.125),
                new Rotation3d(0, 0, Units.degreesToRadians(25))),
            CameraType.HUB_ESTIMATE);
    var turretRight =
        new CameraConfig(
            "turretRight",
            new Transform3d(
                Units.inchesToMeters(-6.194),
                Units.inchesToMeters(-13.0),
                Units.inchesToMeters(20.125),
                new Rotation3d(0, 0, Units.degreesToRadians(-25))),
            CameraType.HUB_ESTIMATE);

    RobotConfig.cameras.add(turretLeft);
    RobotConfig.cameras.add(turretRight);
  }
}
