package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.DriveConstants;
import frc.robot.RobotConfig.OperationMode;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends StateSubsystem<SuperStructureState> {
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
    return getRequestedState();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SuperStructure/state", getCurrentState());
    Logger.recordOutput("SuperStructure/trackingTarget", target);
		Logger.recordOutput("SuperStructure/coolingDown", coolingDown);

    if (coolingDown
        && (getCurrentState() == SuperStructureState.SHOOT
            || shotCooldownTimer.get() > TurretConstants.cooldownSeconds)) {
      coolingDown = false;
    }

    applyState();

    if (FieldConstants.underTrench(RobotState.getInstance().getEstimatedPose())) {
      turret.duck();
    }
  }

  @Override
  public void applyState() {
    SuperStructureState state = getCurrentState();

    turret.track();
    RobotState.getInstance().limitDriveSpeed(DriveConstants.maxDriveSpeedMps);

		if (coolingDown) {
			state = SuperStructureState.SHOOT;
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
        if (state == SuperStructureState.SHOOT) {
          RobotState.getInstance().limitDriveSpeed(DriveConstants.maxSOTMSpeed);
        } else {
          RobotState.getInstance().limitDriveSpeed(DriveConstants.maxSOTMISpeed);
        }
        turret.shoot();

        if (shouldShoot()) {
          intake.agitate();
          if (coolingDown) {
            spindexer.cooldown();
          } else {
            spindexer.feed();
          }

          if (RobotConfig.getMode() == OperationMode.SIM) {
            turret.simulateShot();
          }
        } else {
          intake.idle();
          spindexer.idle();
        }

        if (state == SuperStructureState.SHOOT_INTAKE) {
          intake.run();
        }
        break;
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
