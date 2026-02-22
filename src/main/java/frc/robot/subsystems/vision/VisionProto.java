package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class VisionProto {
  public static final CameraConfig turretRight =
      new CameraConfig(
          "turretRight",
          new Transform3d(
              Units.inchesToMeters(-6.194),
              Units.inchesToMeters(13.0),
              Units.inchesToMeters(20.125),
              new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-25 + 180))),
          RobotConfig.SimConstants.LL4CameraProperties);
  public static final CameraConfig turretLeft =
      new CameraConfig(
          "turretLeft",
          new Transform3d(
              Units.inchesToMeters(-6.194),
              Units.inchesToMeters(-13.0),
              Units.inchesToMeters(20.125),
              new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(25 + 180))),
          RobotConfig.SimConstants.LL4CameraProperties);
  public static final CameraConfig intakeConfig =
      new CameraConfig(
          "intake",
          new Transform3d(
              Units.inchesToMeters(24.806),
              Units.inchesToMeters(13.0 - 3.0),
              Units.inchesToMeters(20.125 - 3.0),
              new Rotation3d()),
          RobotConfig.SimConstants.LL4CameraProperties);

  static {
    RobotConfig.VisionConstants.cameras.add(turretLeft);
    RobotConfig.VisionConstants.cameras.add(turretRight);
  }

  public static void logCameras() {
    var robot = new Pose3d(RobotState.getInstance().getSimulatedPose());
    for (var cam : RobotConfig.VisionConstants.cameras) {
      Logger.recordOutput("VisionProto/" + cam.name(), robot.transformBy(cam.robotToCamera()));
    }
  }
}
