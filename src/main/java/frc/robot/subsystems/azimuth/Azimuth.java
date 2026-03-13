package frc.robot.subsystems.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    return inputs.position.plus(TurretConstants.robotToTurret.getRotation().getMeasureZ());
  }

  public void setAngle(Angle robotAngle) {
    this.io.setAngle(robotAngle.minus(TurretConstants.robotToTurret.getRotation().getMeasureZ()));
  }

  public boolean isCameraAccurate() {
    double positionRots = inputs.position.in(Rotations);
    boolean acc = true;
    if (positionRots < 0 && positionRots > -0.20) acc = false;
    if (positionRots > 0 && positionRots < 0.15) acc = false;
    Logger.recordOutput("Azimuth/cameraEnabled", acc);
    return acc;
  }

  public Pose3d getTurretCameraPose() {
    Angle adjustedAzimuth = getAngle();

    Translation3d cameraOffset =
        new Translation3d(
            new Translation2d(
                TurretConstants.azimuthRadiusMeters, new Rotation2d(adjustedAzimuth)));
    Translation3d totalOffset = TurretConstants.robotToTurret.getTranslation().plus(cameraOffset);
    totalOffset = new Translation3d(totalOffset.getX(), -totalOffset.getY(), totalOffset.getZ());

    Rotation3d totalRotation =
        TurretConstants.cameraRotation.plus(new Rotation3d(0.0, 0.0, adjustedAzimuth.in(Radians)));
    Transform3d transform = new Transform3d(totalOffset, totalRotation);
    return new Pose3d().transformBy(transform);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Azimuth", inputs);
    var pose = getTurretCameraPose();
    var transform = new Transform3d(pose.getTranslation(), pose.getRotation());
    Logger.recordOutput("Azimuth/adjustedPosition", getAngle());
    Logger.recordOutput(
        "Azimuth/TurretCameraPose",
        new Pose3d(RobotState.getInstance().getEstimatedPose()).plus(transform));
    Logger.recordOutput(
        "Azimuth/TurretPose",
        new Pose3d(
            Inches.of(-6),
            Inches.of(-6),
            Inches.of(13.375),
            new Rotation3d(0.0, 0.0, getAngle().in(Radians))));
  }
}
