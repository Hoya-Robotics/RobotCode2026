package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.OperationMode;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.RobotState;
import frc.robot.TurretCalculator;
import frc.robot.TurretCalculator.TurretParameters;
import frc.robot.subsystems.azimuth.Azimuth;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends StateSubsystem<SuperStructureState> {
  private Timer simShotTimer = new Timer();
  private Timer shotCooldownTimer = new Timer();
  private boolean coolingDown = false;
  private TurretParameters cooldownParams =
      new TurretParameters(Radians.of(0.0), Radians.of(0.0), RotationsPerSecond.of(0.0));

  private final Spindexer spindexer;
  private final Hood hood;
  private final Azimuth azimuth;
  private final Launcher launcher;
  private final Intake intake;
  private TurretTarget target;

  public SuperStructure(
      Spindexer spindexer, Hood hood, Azimuth azimuth, Launcher launcher, Intake intake) {
    this.target = TurretTarget.DEFAULT;
    this.spindexer = spindexer;
    this.hood = hood;
    this.azimuth = azimuth;
    this.launcher = launcher;
    this.intake = intake;

    setState(SuperStructureState.IDLE);
    simShotTimer.start();
  }

  @Override
  public void periodic() {
    applyState();
  }

  public boolean isIntaking() {
    return getCurrentState() == SuperStructureState.INTAKE;
  }

  public Command setStateCommand(SuperStructureState state) {
    return Commands.runOnce(() -> setState(state));
  }

  public Command setTarget(TurretTarget target) {
    return Commands.runOnce(() -> this.target = target);
  }

  public Command idle() {
    return Commands.runOnce(() -> setState(SuperStructureState.IDLE));
  }

  public Command intake() {
    return Commands.runOnce(() -> setState(SuperStructureState.INTAKE));
  }

  public Command shoot() {
    return Commands.runOnce(() -> setState(SuperStructureState.SHOOT));
  }

  private void simulateTurretShot(TurretParameters params) {
    if (simShotTimer.get() > 0.25) { // && RobotState.getInstance().consumeFuel()) {
      Pose2d robotPose = RobotState.getInstance().getSimulatedPose();
      Translation3d pos =
          new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).getTranslation();
      double launchSpeedMps =
          params.launcherSpeed().in(RadiansPerSecond)
              * TurretConstants.launcherWheelRadius.in(Meters);

      // Calculate velocity components: hoodAngle=0 is horizontal, 90 deg is straight up
      double hoodAngleRad = Degrees.of(90).minus(params.hoodAngle()).in(Radians);
      double horizontalVel = Math.cos(hoodAngleRad) * launchSpeedMps;
      double verticalVel = Math.sin(hoodAngleRad) * launchSpeedMps;

      // Field-relative yaw = robot rotation + turret azimuth
      double fieldYawRad = robotPose.getRotation().getRadians() + params.azimuthAngle().in(Radians);
      double xVel = horizontalVel * Math.cos(fieldYawRad);
      double yVel = horizontalVel * Math.sin(fieldYawRad);

      Translation3d shotVector = new Translation3d(xVel, yVel, verticalVel);
      RobotState.getInstance().getFuelSim().spawnFuel(pos, shotVector);
      simShotTimer.restart();
    }
  }

  @Override
  public SuperStructureState handleStateTransitions() {
    if (getCurrentState() == null) return getRequestedState();
    SuperStructureState state = getCurrentState();
    switch (state) {
      case SHOOT:
        TurretParameters turretParams =
            TurretCalculator.calculateSetpoints(target, azimuth.getAngle());
        coolingDown = true;
        shotCooldownTimer.restart();
        cooldownParams = turretParams;
        break;
      default:
        break;
    }
    return getRequestedState();
  }

  @Override
  public void applyState() {
    Logger.recordOutput("SuperStructure/state", getCurrentState());
    Logger.recordOutput("SuperStructure/trackingTarget", target);
    TurretParameters turretParams = TurretCalculator.calculateSetpoints(target, azimuth.getAngle());

    azimuth.setAngle(turretParams.azimuthAngle());
    hood.setAngle(Radians.of(0.0));
    launcher.setSpeed(RotationsPerSecond.of(10.0)); // idle speed

    // TODO: hood down on trench
    SuperStructureState state = getCurrentState();
    if (coolingDown && shotCooldownTimer.get() > TurretConstants.cooldownSeconds) {
      shotCooldownTimer.stop();
      coolingDown = false;
    }

    switch (state) {
      case REVERSE_INTAKE:
        intake.reverse();
        spindexer.hold();
        break;
      case IDLE:
        intake.retract();
        spindexer.hold();
        break;
      case INTAKE:
        intake.run();
        spindexer.hold();
        break;
      case SHOOT_INTAKE:
      case SHOOT:
        hood.setAngle(turretParams.hoodAngle());
        launcher.setSpeed(turretParams.launcherSpeed());

        boolean hoodWithinTolerance =
            hood.getAngle().isNear(turretParams.hoodAngle(), TurretConstants.hoodTolerance);
        boolean azimuthWithinTolerance =
            azimuth
                .getAngle()
                .isNear(turretParams.azimuthAngle(), TurretConstants.azimuthTolerance);
        boolean upToSpeed =
            launcher
                .getSpeed()
                .isNear(turretParams.launcherSpeed(), TurretConstants.shotSpeedTolerance);

        Logger.recordOutput("SuperStructure/hoodWithinTolerance", hoodWithinTolerance);
        Logger.recordOutput("SuperStructure/azimuthWithinTolerance", azimuthWithinTolerance);
        Logger.recordOutput("SuperStructure/upToSpeed", upToSpeed);

        if (RobotConfig.getMode() == OperationMode.SIM) {
          simulateTurretShot(turretParams);
        }
        if (upToSpeed && hoodWithinTolerance && azimuthWithinTolerance) {
          if (state == SuperStructureState.SHOOT_INTAKE) {
            intake.run();
          } else {
            intake.agitate();
          }
          spindexer.feed();
        }
        break;
    }

    if (coolingDown && state != SuperStructureState.SHOOT) {
      launcher.setSpeed(cooldownParams.launcherSpeed());
      hood.setAngle(cooldownParams.hoodAngle());
      spindexer.cooldown();
    }

		if (FieldConstants.inNeutralZone(RobotState.getInstance().getEstimatedPose())) {
			hood.setAngle(Degrees.of(0.0));
		}
  }
}
