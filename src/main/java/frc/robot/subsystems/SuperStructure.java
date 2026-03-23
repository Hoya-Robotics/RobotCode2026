package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
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
import frc.robot.TurretCalculator.TurretParameters;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends StateSubsystem<SuperStructureState> {
  private Timer simShotTimer = new Timer();
  private Timer shotCooldownTimer = new Timer();
  private boolean coolingDown = false;

  private final Spindexer spindexer;
  private final Turret turret;
  private final Intake intake;
  private TurretTarget target;

  public SuperStructure(Spindexer spindexer, Turret turret, Intake intake) {
    this.target = TurretTarget.DEFAULT;
    this.spindexer = spindexer;
    this.turret = turret;
    this.intake = intake;

    setState(SuperStructureState.IDLE);
    simShotTimer.start();
  }

  public Command setStateCommand(SuperStructureState state) {
    return Commands.runOnce(() -> setState(state));
  }

  public Command setTargetCommand(TurretTarget target) {
    return Commands.runOnce(() -> this.target = target);
  }

  @Override
  public SuperStructureState handleStateTransitions() {
    if (getCurrentState() == null) return getRequestedState();
    if (getCurrentState() == SuperStructureState.SHOOT
        && getRequestedState() != SuperStructureState.SHOOT) {
      coolingDown = true;
      shotCooldownTimer.restart();
    }
    if (coolingDown && getRequestedState() == SuperStructureState.SHOOT) {
      coolingDown = false;
    }
    return getRequestedState();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SuperStructure/state", getCurrentState());
    Logger.recordOutput("SuperStructure/trackingTarget", target);

    if (coolingDown && shotCooldownTimer.get() > TurretConstants.cooldownSeconds) {
      shotCooldownTimer.stop();
      coolingDown = false;
    }

    applyState();

    if (coolingDown) {
      turret.cooldown();
      spindexer.cooldown();
    }

    if (FieldConstants.underTrench(RobotState.getInstance().getEstimatedPose())) {
      turret.duck();
    }
  }

  @Override
  public void applyState() {
    SuperStructureState state = getCurrentState();

    turret.track();
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
        turret.shoot();
        if (state == SuperStructureState.SHOOT_INTAKE) {
          intake.run();
        }

        /*
        if (RobotConfig.getMode() == OperationMode.SIM
            && hoodWithinTolerance
            && azimuthWithinTolerance) {
          simulateTurretShot(turretParams);
        }
        var chassisSpeeds = RobotState.getInstance().getRobotVelocity();
        if (DriverStation.isAutonomous()
            && Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                < 0.05) {
          underTrench = false;
        }*/

        if (shouldShoot()) {
          if (state != SuperStructureState.SHOOT_INTAKE) {
            intake.agitate();
          }
          spindexer.feed();
          if (RobotConfig.getMode() == OperationMode.SIM) {
            simulateTurretShot(turret.getParameters());
          }
        } else {
          intake.idle();
          spindexer.idle();
        }
        break;
    }
  }

  private void simulateTurretShot(TurretParameters params) {
    if (simShotTimer.get() > 0.25) { // && RobotState.getInstance().consumeFuel()) {
      Pose2d robotPose = RobotState.getInstance().getSimulatedPose();
      Translation3d pos =
          new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).getTranslation();
      // double launchSpeedMps = params.launcherSpeed().in(RadiansPerSecond) *
      // TurretConstants.launcherWheelRadius.in(Meters);
      LinearVelocity launchSpeed =
          MetersPerSecond.of(
              TurretConstants.launcherWheelRadius.times(2.0 * Math.PI).in(Meters)
                  * params
                      .launcherSpeed()
                      .minus(RotationsPerSecond.of(4.0))
                      .in(RotationsPerSecond));

      // Calculate velocity components: hoodAngle=0 is horizontal, 90 deg is straight up
      double hoodAngleRad = Degrees.of(90).minus(params.hoodAngle()).in(Radians);
      double horizontalVel = Math.cos(hoodAngleRad) * launchSpeed.in(MetersPerSecond);
      double verticalVel = Math.sin(hoodAngleRad) * launchSpeed.in(MetersPerSecond);

      // Field-relative yaw = robot rotation + turret azimuth
      double fieldYawRad = robotPose.getRotation().getRadians() + params.azimuthAngle().in(Radians);
      double xVel = horizontalVel * Math.cos(fieldYawRad);
      double yVel = horizontalVel * Math.sin(fieldYawRad);

      Translation3d shotVector = new Translation3d(xVel, yVel, verticalVel);
      // RobotState.getInstance().getFuelSim().spawnFuel(pos, shotVector);
      RobotState.getInstance()
          .getFuelSim()
          .launchFuel(
              launchSpeed,
              params.hoodAngle().unaryMinus().plus(Degrees.of(83.0)),
              params.azimuthAngle(),
              Inches.of(18.66694637));
      // fuelSim.launchFuel(null, null, null, null);
      simShotTimer.restart();
    }
  }

  private boolean shouldShoot() {
    boolean underTrench = FieldConstants.underTrench(RobotState.getInstance().getEstimatedPose());
    Logger.recordOutput("SuperStructure/underTrench", underTrench);

    return turret.readyForFeed() && (!underTrench);
  }

  public boolean isIntaking() {
    return getCurrentState() == SuperStructureState.INTAKE
        || getCurrentState() == SuperStructureState.SHOOT_INTAKE;
  }
}
