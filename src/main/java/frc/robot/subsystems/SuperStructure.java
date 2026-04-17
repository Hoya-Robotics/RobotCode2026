package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.OperationMode;
import frc.robot.RobotConfig.SpindexerConstants.SpindexerState;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretConstants.TurretState;
import frc.robot.RobotState;
import frc.robot.TurretCalculator;
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

  private Debouncer feedAtSetpointDebouncer = new Debouncer(0.4, DebounceType.kFalling);

  public SuperStructure(Spindexer spindexer, Turret turret) {
    this.spindexer = spindexer;
    this.turret = turret;

    setState(SuperStructureState.IDLE);
  }

  public Command setStateCommand(SuperStructureState state) {
    return Commands.runOnce(() -> setState(state), this);
  }

  @Override
  public SuperStructureState handleStateTransitions() {
    if (getCurrentState() == null) return getRequestedState();
    if (getRequestedState() == SuperStructureState.PRE_SHOOT
        && getCurrentState() == SuperStructureState.SHOOT) {
      return SuperStructureState.SHOOT;
    }
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

    boolean underTrench = FieldConstants.underTrench(RobotState.getInstance().getEstimatedPose());
    switch (state) {
      case IDLE:
        spindexer.setState(SpindexerState.HOLD);
        break;
      case PRE_SHOOT:
        if (DriverStation.isTeleopEnabled()) RobotState.getInstance().setDriveSOTM(true);
        turret.setState(TurretState.SHOOT);
        spindexer.setState(SpindexerState.COOLDOWN);

        if ((turret.flywheelUpToSpeed()
                && feedAtSetpointDebouncer.calculate(spindexer.isFeedingAtSpeed()))
            || RobotConfig.getMode() == OperationMode.SIM) {
          setState(SuperStructureState.SHOOT);
        }
        break;
      case SHOOT:
        if (DriverStation.isTeleopEnabled()) RobotState.getInstance().setDriveSOTM(true);
        turret.setState(TurretState.SHOOT);

        if (turret.readyForFeed() && !coolingDown && !underTrench) {
          if (RobotConfig.getMode() == OperationMode.SIM) turret.simulateShot();
          spindexer.setState(SpindexerState.FEED);
        } else {
          spindexer.setState(SpindexerState.COOLDOWN);
        }
        break;
    }

    // Trench override
    if (underTrench) {
      turret.setState(TurretState.NEAR_TRENCH);
    }
  }
}
