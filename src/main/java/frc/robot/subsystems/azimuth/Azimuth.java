package frc.robot.subsystems.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public void setAngle(Angle robotAngle) {
    this.io.setAngle(robotAngle.minus(TurretConstants.robotToTurret.getRotation().getMeasureZ()));
  }

  public Pose3d getTurretCameraPose() {
    double azimuthRadians = inputs.position.in(Radians);

    Translation3d cameraOffset =
        new Translation3d(new Translation2d(TurretConstants.azimuthRadiusMeters, azimuthRadians));
    Rotation3d cameraRotation =
        new Rotation3d(0.0, 0.0, azimuthRadians).rotateBy(TurretConstants.cameraRotation);
    return new Pose3d()
        .transformBy(TurretConstants.robotToTurret)
        .transformBy(new Transform3d(cameraOffset, cameraRotation));
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
