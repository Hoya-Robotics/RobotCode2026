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
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.OperationMode;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretConstants.TurretState;
import frc.robot.RobotConfig.TurretConstants.TurretTarget;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState;
import frc.robot.TurretCalculator;
import frc.robot.TurretCalculator.TurretParameters;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends StateSubsystem<TurretState> {
  private final TurretIO io;
  private TurretTarget target = TurretTarget.DEFAULT;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private TurretParameters parameters =
      new TurretParameters(
          Rotations.of(0.0),
          RotationsPerSecond.of(0.0),
          Rotations.of(0.0),
          RotationsPerSecond.of(0.0),
          false);
  private TurretIOOutputs outputs = new TurretIOOutputs();
  private Timer simShotTimer = new Timer();

  // private Debouncer azimuthErrorDebouncer = new Debouncer(0.25);
  private Timer wrapTimer = new Timer();
  private boolean wrapTriggered = false;
  private boolean azimuthReady = false;

  public Turret(TurretIO io) {
    this.io = io;
    setState(TurretState.IDLE_TRACK);
    simShotTimer.start();
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

  public boolean approachingWrapLimit(double decelSeconds) {
    double pos = inputs.azimuthState.nativePosition();
    double vel = inputs.azimuthState.nativeVelocity();
    if (vel > 1e-3) {
      double timeToMax = (TurretConstants.maxAzimuthAngle.in(Rotations) - pos) / vel;
      if (timeToMax < decelSeconds) return true;
    } else if (vel < -1e-3) {
      double timeToMin = (pos - TurretConstants.minAzimuthAngle.in(Rotations)) / (-vel);
      if (timeToMin < decelSeconds) return true;
    }
    return false;
  }

  public boolean readyForFeed() {
    boolean hoodReady =
        getHoodAngle().isNear(parameters.hoodAngle(), TurretConstants.hoodTolerance);
    boolean upToSpeed =
        getShooterSpeed().isNear(parameters.launcherSpeed(), TurretConstants.shotSpeedTolerance);
    boolean simHasFuel =
        RobotConfig.getMode() == OperationMode.SIM ? RobotState.getInstance().consumeFuel() : true;
    boolean azimuthReady =
        getAzimuthAngle().isNear(parameters.azimuthAngle(), TurretConstants.azimuthTolerance);

    if (wrapTriggered && wrapTimer.get() > 0.75) {
      wrapTriggered = false;
    }

    boolean nearWrap = approachingWrapLimit(0.4);

    Logger.recordOutput("Turret/wrapTriggered", wrapTriggered);
    Logger.recordOutput("Turret/approachingWrapLimit", nearWrap);
    Logger.recordOutput("Turret/azimuthGood", azimuthReady);
    Logger.recordOutput("Turret/hoodReady", hoodReady);
    Logger.recordOutput("Turret/upToSpeed", upToSpeed);

    return hoodReady
        && azimuthReady
        && upToSpeed
        && simHasFuel
        && !wrapTriggered
        && !nearWrap
        && (getCurrentState() != TurretState.NEAR_TRENCH);
  }

  @Override
  public void periodic() {
    RobotState.getInstance()
        .getVision()
        .setRobotToCamera(VisionConstants.turretConfig.name(), getRobotToCamera());
    parameters = TurretCalculator.calculateSetpoints(target, getAzimuthAngle());

    if (parameters.wrapTriggered()) {
      wrapTriggered = true;
      wrapTimer.restart();
    }
    /*
    azimuthReady =
        azimuthErrorDebouncer.calculate(
            getAzimuthAngle().isNear(parameters.azimuthAngle(), TurretConstants.azimuthTolerance));*/

    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/state", getCurrentState());
    logMechanisms();

    Robot.batteryLogger.reportCurrentUsage("Turret/Azimuth", inputs.azimuthState.currentAmps());
    Robot.batteryLogger.reportCurrentUsage("Turret/Hood", inputs.hoodState.currentAmps());
    Robot.batteryLogger.reportCurrentUsage("Turret/Flywheel", inputs.shooterState.currentAmps());

    applyState();

    Logger.recordOutput("Turret/hoodSetpoint", outputs.hoodSetpoint.in(Rotations));
    Logger.recordOutput("Turret/azimuthSetpoint", outputs.azimuthSetpoint);
    Logger.recordOutput(
        "Turret/azimuthVelocitySetpoint", outputs.azimuthVelocitySetpoint.in(RotationsPerSecond));
    Logger.recordOutput("Turret/shooterSetpoint", outputs.shooterSetpoint.in(RotationsPerSecond));

    io.applyOutputs(outputs);
  }

  @Override
  public void applyState() {
    outputs.azimuthSetpoint = parameters.azimuthAngle();
    outputs.azimuthVelocitySetpoint = parameters.azimuthVelocity();

    Logger.recordOutput("Turret/azimuthReady", azimuthReady);

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
        outputs.hoodSetpoint = parameters.hoodAngle();
        outputs.shooterSetpoint = parameters.launcherSpeed();
        break;
    }
  }
}
