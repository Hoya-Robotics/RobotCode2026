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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.OperationMode;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import frc.robot.TurretCalculator;
import frc.robot.TurretCalculator.TurretParameters;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum TurretState {
  IDLE_TRACK,
  NEAR_TRENCH,
  SHOOT
}

public class Turret extends StateSubsystem<TurretState> {
  private final TurretIO io;
  private TurretTarget target = TurretTarget.DEFAULT;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private TurretParameters parameters =
      new TurretParameters(
          Rotations.of(0.0),
          RotationsPerSecond.of(0.0),
          Rotations.of(0.0),
          RotationsPerSecond.of(0.0));
  private TurretIOOutputs outputs = new TurretIOOutputs();
  private Timer simShotTimer = new Timer();

  public Turret(TurretIO io) {
    this.io = io;
    setState(TurretState.IDLE_TRACK);
    simShotTimer.start();
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

  public void simulateShot() {
    if (simShotTimer.get() < 0.25) return;
    LinearVelocity launchSpeed =
        MetersPerSecond.of(
            TurretConstants.launcherWheelRadius.times(2.0 * Math.PI).in(Meters)
                * parameters
                    .launcherSpeed()
                    .minus(RotationsPerSecond.of(4.0))
                    .in(RotationsPerSecond));
    RobotState.getInstance()
        .getFuelSim()
        .launchFuel(
            launchSpeed,
            parameters.hoodAngle().unaryMinus().plus(Degrees.of(83.0)),
            parameters.azimuthAngle(),
            Inches.of(18.66694637));
    simShotTimer.restart();
  }

  public Transform3d getRobotToCamera() {
    Translation3d cameraOffset =
        new Translation3d(
            new Translation2d(
                TurretConstants.azimuthRadiusMeters, new Rotation2d(getAzimuthAngle())));
    Translation3d totalOffset = TurretConstants.robotToTurret.getTranslation().plus(cameraOffset);
    totalOffset = new Translation3d(totalOffset.getX(), -totalOffset.getY(), totalOffset.getZ());

    Rotation3d totalRotation =
        TurretConstants.cameraRotation.plus(
            new Rotation3d(0.0, 0.0, getAzimuthAngle().in(Radians)));
    return new Transform3d(totalOffset, totalRotation);
  }

  public boolean readyForFeed() {
    boolean hoodReady =
        getHoodAngle().isNear(parameters.hoodAngle(), TurretConstants.hoodTolerance);
    boolean azimuthReady =
        getAzimuthAngle().isNear(parameters.azimuthAngle(), TurretConstants.azimuthTolerance);
    boolean upToSpeed =
        getShooterSpeed().isNear(parameters.launcherSpeed(), TurretConstants.shotSpeedTolerance);
    boolean simHasFuel =
        RobotConfig.getMode() == OperationMode.SIM ? RobotState.getInstance().consumeFuel() : true;

    Logger.recordOutput("Turret/hoodReady", hoodReady);
    Logger.recordOutput("Turret/azimuthReady", azimuthReady);
    Logger.recordOutput("Turret/upToSpeed", upToSpeed);

    return hoodReady && azimuthReady && upToSpeed && simHasFuel && (!willTurretWrap(0.4));
  }

  public boolean willTurretWrap(double dt) {
    double pos = inputs.azimuthState.nativePosition();
    double robotContrib =
        RadiansPerSecond.of(RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond)
            .in(RotationsPerSecond);
    double futurePos =
        inputs.azimuthState.nativePosition()
            + (inputs.azimuthState.nativeVelocity() + robotContrib) * dt;
    boolean tooFar =
        pos < TurretConstants.maxAzimuthAngle.in(Rotations)
            && futurePos > TurretConstants.maxAzimuthAngle.in(Rotations);
    boolean tooShort =
        pos > TurretConstants.minAzimuthAngle.in(Rotations)
            && futurePos < TurretConstants.minAzimuthAngle.in(Rotations);

    return tooFar || tooShort;
  }

  @Override
  public void periodic() {
    RobotState.getInstance()
        .getVision()
        .setRobotToCamera(VisionConstants.turretConfig.name(), getRobotToCamera());
    parameters = TurretCalculator.calculateSetpoints(target, getAzimuthAngle());

    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/state", getCurrentState());
    logMechanisms();

    applyState();

    Logger.recordOutput("Turret/hoodSetpoint", outputs.hoodSetpoint.in(Degrees));
    Logger.recordOutput("Turret/azimuthSetpoint", outputs.azimuthSetpoint);
    Logger.recordOutput("Turret/shooterSetpoint", outputs.shooterSetpoint);

    io.applyOutputs(outputs);
  }

  @Override
  public void applyState() {
    outputs.azimuthSetpoint = parameters.azimuthAngle();
    outputs.azimuthVelocitySetpoint = parameters.azimuthVelocity();
    switch (getCurrentState()) {
      case IDLE_TRACK:
        outputs.shooterSetpoint = TurretConstants.shotIdleSpeed;
        outputs.hoodSetpoint = parameters.hoodAngle();
        break;
      case NEAR_TRENCH:
        outputs.shooterSetpoint = TurretConstants.shotIdleSpeed;
        outputs.hoodSetpoint =
            Degrees.of(
                Math.min(
                    parameters.hoodAngle().in(Degrees),
                    TurretConstants.trenchHoodAngle.in(Degrees)));
        break;
      case SHOOT:
        outputs.shooterSetpoint = parameters.launcherSpeed();
        outputs.hoodSetpoint = parameters.hoodAngle();
        break;
    }
  }
}
