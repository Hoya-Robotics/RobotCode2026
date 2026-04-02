package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
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
import frc.robot.util.LoggedTunableNumber;
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

  private final LoggedTunableNumber flywheelIdleSpeed =
      new LoggedTunableNumber("Turret/Flywheel/idleSpeedRPS", 10);
  private final LoggedTunableNumber wrapLooheadSeconds =
      new LoggedTunableNumber("Turret/Azimuth/wrapLooheadSeconds", 0.5);

  private final LinearFilter azimuthFFFilter = LinearFilter.movingAverage(5);
  private final Debouncer azimuthSettledDebouncer = new Debouncer(0.1, DebounceType.kRising);

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

  private boolean willAzimuthWrapWithin(double dt) {
    double posRots = inputs.azimuthState.nativePosition();
    double futurePos = posRots + dt * inputs.azimuthState.nativeVelocity();
    return futurePos > TurretConstants.maxAzimuthAngle.in(Rotations)
        || futurePos < TurretConstants.minAzimuthAngle.in(Rotations);
  }

  private boolean isAzimuthTracking() {
    double posError = Math.abs(inputs.azimuthState.nativePosition() - outputs.azimuthSetpointRots);
    double velError = Math.abs(inputs.azimuthState.nativeVelocity() - outputs.azimuthFFRotsPerSec);

    boolean withinTolerance = posError < 0.02 && velError < 0.05;
    return azimuthSettledDebouncer.calculate(withinTolerance);
  }

  public boolean readyForFeed() {
    boolean hoodReady =
        getHoodAngle().isNear(parameters.hoodAngle(), TurretConstants.hoodTolerance);
    boolean upToSpeed =
        getShooterSpeed().isNear(parameters.launcherSpeed(), TurretConstants.shotSpeedTolerance);
    boolean simHasFuel =
        RobotConfig.getMode() == OperationMode.SIM ? RobotState.getInstance().consumeFuel() : true;
    boolean azimuthReady = isAzimuthTracking();
    boolean willWrap = willAzimuthWrapWithin(wrapLooheadSeconds.getAsDouble());

    Logger.recordOutput("Turret/Ready/azimuthSettled", azimuthReady);
    Logger.recordOutput("Turret/Ready/hoodPosition", hoodReady);
    Logger.recordOutput("Turret/Ready/flywheelSpeed", upToSpeed);
    Logger.recordOutput("Turret/Ready/azimuthWillWrap", willWrap);

    return hoodReady
        && azimuthReady
        && upToSpeed
        && simHasFuel
        && (!willWrap)
        && (getCurrentState() != TurretState.NEAR_TRENCH);
  }

  @Override
  public void periodic() {
    RobotState.getInstance()
        .getVision()
        .setRobotToCamera(VisionConstants.turretConfig.name(), getRobotToCamera());
    parameters = TurretCalculator.calculateSetpoints(target, getAzimuthAngle());

    io.updateInputs(inputs);

    // Logging
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/state", getCurrentState());
    logMechanisms();

    Robot.batteryLogger.reportCurrentUsage("Turret/Azimuth", inputs.azimuthState.currentAmps());
    Robot.batteryLogger.reportCurrentUsage("Turret/Hood", inputs.hoodState.currentAmps());
    Robot.batteryLogger.reportCurrentUsage("Turret/Flywheel", inputs.shooterState.currentAmps());

    // Logic
    applyState();
    io.applyOutputs(outputs);

    Logger.recordOutput("Turret/Setpoints/hoodRots", outputs.hoodSetpointRots);
    Logger.recordOutput("Turret/Setpoints/azimuthRots", outputs.azimuthSetpointRots);
    Logger.recordOutput("Turret/Setpoints/azimuthFFRPS", outputs.azimuthFFRotsPerSec);
    Logger.recordOutput("Turret/Setpoints/flywheelRPS", outputs.flywheelRPS);
  }

  @Override
  public void applyState() {
    outputs.azimuthSetpointRots = parameters.azimuthAngle().in(Rotations);
    outputs.azimuthFFRotsPerSec =
        azimuthFFFilter.calculate(parameters.azimuthVelocity().in(RotationsPerSecond));
    outputs.flywheelRPS = flywheelIdleSpeed.getAsDouble();
    outputs.hoodSetpointRots = parameters.hoodAngle().in(Rotations);

    switch (getCurrentState()) {
      case NEAR_TRENCH:
        outputs.hoodSetpointRots =
            Math.min(
                parameters.hoodAngle().in(Rotations),
                TurretConstants.trenchHoodAngle.in(Rotations));
        break;
      case SHOOT:
        outputs.flywheelRPS = parameters.launcherSpeed().in(RotationsPerSecond);
        break;
      default:
        break;
    }
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
}
