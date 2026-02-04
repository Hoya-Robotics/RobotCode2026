package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionProto {
  private static final CameraConfig turretRight =
      new CameraConfig(
          "turretRight",
          new Transform3d(
              Units.inchesToMeters(-6.194),
              Units.inchesToMeters(13.0),
              Units.inchesToMeters(20.125),
              new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-25 + 180))),
          CameraType.HUB_ESTIMATE);
  private static final CameraConfig turretLeft =
      new CameraConfig(
          "turretLeft",
          new Transform3d(
              Units.inchesToMeters(-6.194),
              Units.inchesToMeters(-13.0),
              Units.inchesToMeters(20.125),
              new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(25 + 180))),
          CameraType.HUB_ESTIMATE);

  private static final CameraConfig intakeLocalizer =
      new CameraConfig(
          "intake",
          new Transform3d(
              Units.inchesToMeters(24.806),
              Units.inchesToMeters(13.0 - 3.0),
              Units.inchesToMeters(20.125 - 3.0),
              new Rotation3d()),
          CameraType.HUB_ESTIMATE);

  public static void logCameras() {
    var robot = new Pose3d(RobotState.getInstance().getSimulatedDrivePose());
    for (var cam : RobotConfig.VisionConstants.cameras) {
      Logger.recordOutput("VisionProto/" + cam.name(), robot.transformBy(cam.robotToCamera()));
    }
  }

  public static LocalizationCameraIO[] wideFOVFrontSetup() {
    RobotConfig.VisionConstants.cameras.add(turretLeft);
    RobotConfig.VisionConstants.cameras.add(turretRight);

    List<LocalizationCameraIO> localizers = new ArrayList<>();
    localizers.add(new PhotonSimLocalizationCamera(turretLeft));
    localizers.add(new PhotonSimLocalizationCamera(turretRight));

    return localizers.toArray(LocalizationCameraIO[]::new);
  }

  public static LocalizationCameraIO[] wideFOVFrontSetupWithIntakeLocalizer() {
    RobotConfig.VisionConstants.cameras.add(turretLeft);
    RobotConfig.VisionConstants.cameras.add(turretRight);
    RobotConfig.VisionConstants.cameras.add(intakeLocalizer);

    List<LocalizationCameraIO> localizers = new ArrayList<>();
    localizers.add(new PhotonSimLocalizationCamera(turretLeft));
    localizers.add(new PhotonSimLocalizationCamera(turretRight));
    localizers.add(new PhotonSimLocalizationCamera(intakeLocalizer));

    return localizers.toArray(LocalizationCameraIO[]::new);
  }
}
