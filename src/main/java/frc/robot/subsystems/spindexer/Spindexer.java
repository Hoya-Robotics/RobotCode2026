package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum SpindexerState {
  HOLD,
  FEED,
  COOLDOWN,
  REVERSE
}

public class Spindexer extends StateSubsystem<SpindexerState> {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private SpindexerIOOutputs outputs = new SpindexerIOOutputs();
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

    applyState();

    Logger.recordOutput("Spindexer/state", getCurrentState());
    io.applyOutputs(outputs);
  }

  public void hold() {
    setState(SpindexerState.HOLD);
  }

  public void feed() {
    setState(SpindexerState.FEED);
  }

  public void cooldown() {
    setState(SpindexerState.COOLDOWN);
  }

  public void reverse() {
    setState(SpindexerState.REVERSE);
  }

  private boolean isStalled() {
    return stateChangeTimer.get() > 0.25 && (inputs.feedMotorVelocity.abs(RPM) < 1.75 || inputs.indexMotorVelocity.abs(RPM) < 1.75);
  }

	@Override
	public SpindexerState handleStateTransitions() {
		if (getRequestedState() != getCurrentState()) {
			stateChangeTimer.restart();
		}
		return getRequestedState();
	}

  @Override
  public void applyState() {
    if (unjamming && unjamTimer.get() > 0.35) {
      unjamming = false;
    } else if (!unjamming && getCurrentState() == SpindexerState.FEED && isStalled()) {
      unjamming = true;
      unjamTimer.restart();
    }

    if (unjamming) {
      setState(SpindexerState.REVERSE);
    }

    switch (getCurrentState()) {
      case HOLD:
        outputs.indexMotorVoltage = Volts.zero();
        outputs.feedMotorVoltage = Volts.zero();
        break;
      case COOLDOWN:
        outputs.indexMotorVoltage = Volts.of(0.0);
        outputs.feedMotorVoltage = Volts.of(4.5);
        break;
      case FEED:
        outputs.indexMotorVoltage = Volts.of(2.5);
        outputs.feedMotorVoltage = Volts.of(4.5);
        break;
      case REVERSE:
        outputs.indexMotorVoltage = Volts.of(-4.5);
        outputs.feedMotorVoltage = Volts.of(-4.5);
        break;
    }
  }
}
