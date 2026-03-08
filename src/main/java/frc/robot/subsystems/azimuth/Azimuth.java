package frc.robot.subsystems.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Azimuth extends SubsystemBase {
  private final AzimuthIO io;
  private final AzimuthIOInputsAutoLogged inputs = new AzimuthIOInputsAutoLogged();

  public Azimuth(AzimuthIO io) {
    this.io = io;
  }

  public Angle getAngle() {
    return inputs.position;
  }

  public void setAngle(Angle angle) {
    this.io.setAngle(angle);
  }

  public Pose3d getTurretCameraPose() {
    double azimuthRadians = inputs.position.in(Radians);

    // Get turret center position in robot space
    double turretX = TurretConstants.robotToTurret.getX();
    double turretY = TurretConstants.robotToTurret.getY();
    double turretZ = TurretConstants.robotToTurret.getZ();

    // Camera position relative to turret center, rotated by azimuth
    // The camera orbits around the turret pivot at azimuthRadiusMeters
    double cameraLocalX =
        TurretConstants.turretToCamera.getX()
            + TurretConstants.azimuthRadiusMeters * Math.cos(azimuthRadians);
    double cameraLocalY =
        TurretConstants.turretToCamera.getY()
            + TurretConstants.azimuthRadiusMeters * Math.sin(azimuthRadians);
    double cameraLocalZ = TurretConstants.turretToCamera.getZ();

    // Final camera position in robot space
    double cameraX =
        turretX + cameraLocalX * Math.cos(azimuthRadians) - cameraLocalY * Math.sin(azimuthRadians);
    double cameraY =
        turretY + cameraLocalX * Math.sin(azimuthRadians) + cameraLocalY * Math.cos(azimuthRadians);
    double cameraZ = turretZ + cameraLocalZ;

    // Camera rotation: azimuth yaw + fixed pitch/roll offsets
    Rotation3d cameraRotation =
        new Rotation3d(
            TurretConstants.turretToCamera.getRotation().getX(),
            TurretConstants.turretToCamera.getRotation().getY(),
            azimuthRadians
                + TurretConstants.turretCameraMagicOffset.in(Radians)
                + TurretConstants.robotToTurret.getRotation().getZ());

    return new Pose3d(cameraX, cameraY, cameraZ, cameraRotation);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Azimuth", inputs);
    var pose = getTurretCameraPose();
    var transform = new Transform3d(pose.getTranslation(), pose.getRotation());
    Logger.recordOutput(
        "Azimuth/TurretCameraPose",
        new Pose3d(RobotState.getInstance().getEstimatedPose()).transformBy(transform));
  }
}
