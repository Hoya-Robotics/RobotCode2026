package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotConfig.SpindexerConstants.SpindexerState;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends StateSubsystem<SpindexerState> {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private SpindexerIOOutputs outputs = new SpindexerIOOutputs();

  private LoggedTunableNumber indexSpeed = new LoggedTunableNumber("Spindexer/indexSpeed", 14.0);
  private LoggedTunableNumber feedSpeed = new LoggedTunableNumber("Spindexer/feedSpeed", 22.0);

  private boolean unjamming = false;
  private Timer unjamTimer = new Timer();
  private Timer stateChangeTimer = new Timer();

  public Spindexer(SpindexerIO io) {
    this.io = io;
    setState(SpindexerState.HOLD);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);

    Robot.batteryLogger.reportCurrentUsage("Spindexer/Indexer", inputs.indexMotorCurrent.in(Amps));
    Robot.batteryLogger.reportCurrentUsage("Spindexer/Feeder", inputs.feedMotorCurrent.in(Amps));

    // Update unjamming state
    if (unjamming && unjamTimer.get() > 0.45) {
      unjamming = false;
    } else if (!unjamming && getCurrentState() == SpindexerState.FEED && isStalled()) {
      unjamming = true;
      unjamTimer.restart();
    }

    if (unjamming) {
      setState(SpindexerState.REVERSE);
    }

    applyState();

    Logger.recordOutput("Spindexer/state", getCurrentState());
    io.applyOutputs(outputs);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case HOLD:
        outputs.indexMotorVelocity = RevolutionsPerSecond.of(0.0);
        outputs.feedVelocity = RotationsPerSecond.of(0.0);
        break;
      case COOLDOWN:
        outputs.indexMotorVelocity = RevolutionsPerSecond.of(0.0);
        outputs.feedVelocity = RotationsPerSecond.of(feedSpeed.getAsDouble());
        break;
      case FEED:
        outputs.feedVelocity = RotationsPerSecond.of(feedSpeed.getAsDouble());
        outputs.indexMotorVelocity = RevolutionsPerSecond.of(indexSpeed.getAsDouble());
        break;
      case REVERSE:
        outputs.indexMotorVelocity = RevolutionsPerSecond.of(-10.0);
        outputs.feedVelocity = RotationsPerSecond.of(20.0);
        break;
    }
  }

  private boolean isStalled() {
    return false;
    /*
    return stateChangeTimer.get() > 0.25
        && (inputs.feedMotorVelocity_velocityRotationsPerSecond.abs(RPM) < 2.0
            || inputs.indexMotorVelocity_velocityRotationsPerSecond.abs(RPM) < 2.0);*/
  }

  @Override
  public SpindexerState handleStateTransitions() {
    if (getRequestedState() != getCurrentState() && getCurrentState() != SpindexerState.REVERSE) {
      stateChangeTimer.restart();
    }
    return getRequestedState();
  }
}
