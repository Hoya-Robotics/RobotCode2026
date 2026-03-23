package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.TurretCalculator;
import frc.robot.TurretCalculator.TurretParameters;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum TurretState {
  IDLE_TRACK,
  NEAR_TRENCH,
  COOLDOWN,
  SHOOT
}

public class Turret extends StateSubsystem<TurretState> {
  private final TurretIO io;
  private TurretTarget target = TurretTarget.DEFAULT;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private TurretParameters parameters =
      new TurretParameters(Rotations.of(0.0), Rotations.of(0.0), RotationsPerSecond.of(0.0));
  private TurretParameters cooldownParams;
  private TurretIOOutputs outputs = new TurretIOOutputs();

  public Turret(TurretIO io) {
    this.io = io;
    setState(TurretState.IDLE_TRACK);
  }

  public void cooldown() {
    setState(TurretState.COOLDOWN);
  }

  public void track() {
    setState(TurretState.IDLE_TRACK);
  }

  public void shoot() {
    setState(TurretState.SHOOT);
  }

  public void duck() {
    setState(TurretState.NEAR_TRENCH);
  }

  public Angle getAzimuthAngle() {
    return Rotations.of(inputs.azimuthState.nativePosition());
  }

  public Angle getHoodAngle() {
    return Rotations.of(inputs.hoodState.nativePosition());
  }

  public AngularVelocity getShooterSpeed() {
    return RotationsPerSecond.of(inputs.shooterState.nativeVelocity());
  }

  public TurretParameters getParameters() {
    return parameters;
  }

  public void setTarget(TurretTarget target) {
    this.target = target;
  }

  private void logMechanisms() {
    Logger.recordOutput(
        "Turret/HoodPose",
        new Pose3d(
                Inches.of(-6),
                Inches.of(-6),
                Inches.of(18.66694637),
                new Rotation3d(Rotations.zero(), getHoodAngle(), getAzimuthAngle()))
            .transformBy(
                new Transform3d(
                    Inches.of(3.77617094), Inches.zero(), Inches.zero(), Rotation3d.kZero)));
    Logger.recordOutput(
        "Turret/AzimuthPose",
        new Pose3d(
            Inches.of(-6),
            Inches.of(-6),
            Inches.of(13.375),
            new Rotation3d(0.0, 0.0, getAzimuthAngle().in(Radians))));
  }

  public Pose3d getCameraPose() {
    Translation3d cameraOffset =
        new Translation3d(
            new Translation2d(
                TurretConstants.azimuthRadiusMeters, new Rotation2d(getAzimuthAngle())));
    Translation3d totalOffset = TurretConstants.robotToTurret.getTranslation().plus(cameraOffset);
    totalOffset = new Translation3d(totalOffset.getX(), -totalOffset.getY(), totalOffset.getZ());

    Rotation3d totalRotation =
        TurretConstants.cameraRotation.plus(
            new Rotation3d(0.0, 0.0, getAzimuthAngle().in(Radians)));
    Transform3d transform = new Transform3d(totalOffset, totalRotation);
    return new Pose3d().transformBy(transform);
  }

  public boolean readyForFeed() {
    boolean hoodReady =
        getHoodAngle().isNear(parameters.hoodAngle(), TurretConstants.hoodTolerance);
    boolean azimuthReady =
        getAzimuthAngle().isNear(parameters.azimuthAngle(), TurretConstants.azimuthTolerance);
    boolean upToSpeed =
        getShooterSpeed().isNear(parameters.launcherSpeed(), TurretConstants.shotSpeedTolerance);

    Logger.recordOutput("Turret/hoodReady", hoodReady);
    Logger.recordOutput("Turret/azimuthReady", azimuthReady);
    Logger.recordOutput("Turret/upToSpeed", upToSpeed);

    return hoodReady && azimuthReady && upToSpeed;
  }

  public TurretParameters getTurretParameters() {
    return parameters;
  }

  @Override
  public void periodic() {
    parameters = TurretCalculator.calculateSetpoints(target, getAzimuthAngle());

    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    Logger.recordOutput("Turret/hoodSetpoint", outputs.hoodSetpoint);
    Logger.recordOutput("Turret/azimuthSetpoint", outputs.azimuthSetpoint);
    Logger.recordOutput("Turret/shooterSetpoint", outputs.shooterSetpoint);

    logMechanisms();

    applyState();
    io.applyOutputs(outputs);
  }

  @Override
  public TurretState handleStateTransitions() {
    if (getRequestedState() == TurretState.COOLDOWN && getCurrentState() != TurretState.COOLDOWN)
      cooldownParams = parameters;
    return getRequestedState();
  }

  @Override
  public void applyState() {
    outputs.azimuthSetpoint = parameters.azimuthAngle();
    switch (getCurrentState()) {
      case IDLE_TRACK:
        outputs.shooterSetpoint = TurretConstants.shotIdleSpeed;
        outputs.hoodSetpoint = parameters.hoodAngle();
        break;
      case NEAR_TRENCH:
        outputs.shooterSetpoint = TurretConstants.shotIdleSpeed;
        outputs.hoodSetpoint = Degrees.of(0.0);
        break;
      case SHOOT:
        outputs.shooterSetpoint = parameters.launcherSpeed();
        outputs.hoodSetpoint = parameters.hoodAngle();
        break;
      case COOLDOWN:
        outputs.shooterSetpoint = cooldownParams.launcherSpeed();
        outputs.hoodSetpoint = cooldownParams.hoodAngle();
        break;
    }
  }
}
