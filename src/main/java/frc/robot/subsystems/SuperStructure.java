package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.IntakeConstants.IntakeState;
import frc.robot.RobotConfig.OperationMode;
import frc.robot.RobotConfig.SpindexerConstants.SpindexerState;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretConstants.TurretState;
import frc.robot.RobotState;
import frc.robot.TurretCalculator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlip;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends StateSubsystem<SuperStructureState> {
  private Timer shotCooldownTimer = new Timer();
  private boolean coolingDown = false;

  private final Spindexer spindexer;
  private final Turret turret;
  private final Intake intake;

  public SuperStructure(Spindexer spindexer, Turret turret, Intake intake) {
    this.spindexer = spindexer;
    this.turret = turret;
    this.intake = intake;

    setState(SuperStructureState.IDLE);
  }

  public Command setStateCommand(SuperStructureState state) {
    return Commands.runOnce(() -> setState(state), this);
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
    Logger.recordOutput("SuperStructure/coolingDown", coolingDown);

    if (coolingDown
        && (getCurrentState() == SuperStructureState.SHOOT
            || shotCooldownTimer.get() > TurretConstants.cooldownSeconds)) {
      coolingDown = false;
    }

    applyState();
  }

  @Override
  public void applyState() {
    SuperStructureState state = getCurrentState();

    // Turret targeting
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (FieldConstants.inAllianceZone(robotPose)) {
      turret.setTarget(AllianceFlip.apply(FieldConstants.hubCenter.toTranslation2d()), false);
    } else {
      turret.setTarget(AllianceFlip.apply(TurretCalculator.getPassingTarget()), true);
    }

    turret.setState(TurretState.IDLE_TRACK);
    RobotState.getInstance().setDriveSOTM(false);

    // State guards
    if (coolingDown) {
      state = SuperStructureState.SHOOT;
    }

    switch (state) {
      case REVERSE_INTAKE:
        intake.setState(IntakeState.REVERSE);
        spindexer.setState(SpindexerState.HOLD);
        break;
      case IDLE:
        intake.setState(IntakeState.IDLE);
        spindexer.setState(SpindexerState.HOLD);
        break;
      case INTAKE:
        intake.setState(IntakeState.INTAKE);
        spindexer.setState(SpindexerState.HOLD);
        break;
      case SHOOT_INTAKE:
      case SHOOT:
        if (DriverStation.isTeleopEnabled()) RobotState.getInstance().setDriveSOTM(true);
        turret.setState(TurretState.SHOOT);

        if (turret.readyForFeed()) {
          intake.setState(IntakeState.AGITATE);

          if (coolingDown) {
            spindexer.setState(SpindexerState.COOLDOWN);
          } else {
            spindexer.setState(SpindexerState.FEED);
          }

          if (RobotConfig.getMode() == OperationMode.SIM) {
            turret.simulateShot();
          }
        } else {
          intake.setState(IntakeState.IDLE);
          spindexer.setState(SpindexerState.HOLD);
        }

        if (state == SuperStructureState.SHOOT_INTAKE) {
          intake.setState(IntakeState.INTAKE);
        }
        break;
    }

    // Trench override
    if (FieldConstants.underTrench(RobotState.getInstance().getEstimatedPose())) {
      turret.setState(TurretState.NEAR_TRENCH);
    }
  }
}
