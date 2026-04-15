package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretConstants.TurretState;
import frc.robot.RobotState;
import frc.robot.TurretCalculator;
import frc.robot.TurretCalculator.TurretParameters;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends StateSubsystem<TurretState> {
  private final TurretIO io;
  private Translation2d target = Translation2d.kZero;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private TurretParameters parameters =
      new TurretParameters(
          Rotations.of(0.0),
          RotationsPerSecond.of(0.0),
          Rotations.of(0.0),
          RotationsPerSecond.of(0.0));
  private TurretIOOutputs outputs = new TurretIOOutputs();
  private Timer simShotTimer = new Timer();
  private boolean passing = false;

  private final LoggedTunableNumber flywheelIdleSpeed =
      new LoggedTunableNumber("Turret/Flywheel/idleSpeedRPS", 10);
  private final LoggedTunableNumber kVelocity =
      new LoggedTunableNumber("Turret/Azimuth/kVelocity", 0.0);
  private final LoggedTunableNumber wrapMarginDegs =
      new LoggedTunableNumber("Turret/Azimuth/wrapMarginDegs", 8);
  private final LoggedTunableNumber flywheelTuningSpeed =
      new LoggedTunableNumber("Turret/Flywheel/tuningSetpoint", 40);
  private final LoggedTunableNumber hoodTuningSetpoint =
      new LoggedTunableNumber("Turret/Flywheel/hoodSetpoint", 10);

  private final Debouncer azimuthSettledDebouncer = new Debouncer(0.25, DebounceType.kFalling);

  private double lastTime = 0.0;
  private double azimuthFFVel = 0.0;

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
    return RotationsPerSecond.of(
        Math.min(
            inputs.leftFlywheelState.nativeVelocity(), inputs.rightFlywheelState.nativeVelocity()));
  }

  public TurretParameters getParameters() {
    return parameters;
  }

  public void setTarget(Translation2d target, boolean passing) {
    this.target = target;
    this.passing = passing;
  }

  private boolean nearWrapBoundary() {
    double pos = inputs.azimuthState.nativePosition();
    double max = TurretConstants.maxAzimuthAngle.in(Rotations);
    double min = TurretConstants.minAzimuthAngle.in(Rotations);
    double WRAP_MARGIN_ROTS = Units.degreesToRotations(wrapMarginDegs.getAsDouble());
    return (max - pos) < WRAP_MARGIN_ROTS || (pos - min) < WRAP_MARGIN_ROTS;
  }

  public boolean flywheelUpToSpeed() {
    return Math.max(
            Math.abs(inputs.leftFlywheelState.nativeVelocity() - outputs.flywheelRPS),
            Math.abs(inputs.rightFlywheelState.nativeVelocity() - outputs.flywheelRPS))
        < 3.0;
  }

  public boolean readyForFeed() {
    double posError = Math.abs(inputs.azimuthState.nativePosition() - outputs.azimuthSetpointRots);
    ChassisSpeeds speeds = RobotState.getInstance().getFieldVelocity();
    double tolerance =
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 0.2
            ? TurretConstants.azimuthMovingToleranceRots
            : TurretConstants.azimuthStaticToleranceRots;
    boolean withinTolerance = posError < tolerance;
    boolean azimuthAtSetpoint = azimuthSettledDebouncer.calculate(withinTolerance);
    boolean willWrap = nearWrapBoundary();

    boolean ready =
        azimuthAtSetpoint && (!willWrap) && (getCurrentState() != TurretState.NEAR_TRENCH);

    Logger.recordOutput("Turret/Ready/azimuthAtSetpoint", azimuthAtSetpoint);
    Logger.recordOutput("Turret/Ready/azimuthWillWrap", willWrap);
    Logger.recordOutput("Turret/Ready/fullyReady", ready);

    return ready;
  }

  @Override
  public void periodic() {
    parameters = TurretCalculator.turretIterativeMovingSetpoint(target, passing, getAzimuthAngle());

    io.updateInputs(inputs);

    // Logging
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/state", getCurrentState());
    Logger.recordOutput("Turret/target", target);
    logMechanisms();

    Robot.batteryLogger.reportCurrentUsage("Turret/Azimuth", inputs.azimuthState.currentAmps());
    Robot.batteryLogger.reportCurrentUsage("Turret/Hood", inputs.hoodState.currentAmps());
    Robot.batteryLogger.reportCurrentUsage(
        "Turret/FlywheelL", inputs.leftFlywheelState.currentAmps());
    Robot.batteryLogger.reportCurrentUsage(
        "Turret/FlywheelR", inputs.rightFlywheelState.currentAmps());

    // Logic
    double lastAzimuthSetpoint = outputs.azimuthSetpointRots;
    applyState();

    // Calculate azimuth feedforward term
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    if (dt > 0.0) {
      azimuthFFVel = (outputs.azimuthSetpointRots - lastAzimuthSetpoint) / dt;
    }
    lastTime = currentTime;
    outputs.azimuthFeedforward = azimuthFFVel * kVelocity.getAsDouble();

    io.applyOutputs(outputs);

    Logger.recordOutput("Turret/Setpoints/hoodRots", outputs.hoodSetpointRots);
    Logger.recordOutput("Turret/Setpoints/azimuthRots", outputs.azimuthSetpointRots);
    Logger.recordOutput("Turret/Setpoints/azimuthFFRPS", outputs.azimuthFeedforward);
    Logger.recordOutput("Turret/Setpoints/flywheelRPS", outputs.flywheelRPS);
  }

  @Override
  public void applyState() {
    outputs.azimuthSetpointRots = parameters.azimuthAngle().in(Rotations);
    outputs.flywheelRPS = flywheelIdleSpeed.getAsDouble();
    outputs.hoodSetpointRots = MathUtil.clamp(parameters.hoodAngle().in(Rotations), 0.02, 0.078);

    switch (getCurrentState()) {
      case NEAR_TRENCH:
        outputs.hoodSetpointRots =
            Math.min(
                parameters.hoodAngle().in(Rotations),
                TurretConstants.trenchHoodAngle.in(Rotations));
        break;
      case SHOOT:
        outputs.flywheelRPS = parameters.flywheelSpeed().in(RotationsPerSecond);
        break;
      default:
        break;
    }
    /*
      outputs.flywheelRPS = flywheelTuningSpeed.getAsDouble();
      outputs.hoodSetpointRots = Units.degreesToRotations(hoodTuningSetpoint.getAsDouble());
    */
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
    if (!RobotState.getInstance().consumeFuel()) return;
    LinearVelocity launchSpeed =
        MetersPerSecond.of(
            TurretConstants.launcherWheelRadius.times(2.0 * Math.PI).in(Meters)
                * parameters
                    .flywheelSpeed()
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
}
