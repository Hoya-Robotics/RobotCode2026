package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum SpindexerState {
  HOLD,
  FEED,
  REVERSE
}

public class Spindexer extends StateSubsystem<SpindexerState> {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private SpindexerIOOutputs outputs = new SpindexerIOOutputs();

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

  public void reverse() {
    setState(SpindexerState.REVERSE);
  }

  private boolean isStalled() {
    return inputs.feedMotorVelocity.abs(RPM) < 1.75 || inputs.indexMotorVelocity.abs(RPM) < 1.75;
  }

  @Override
  public void applyState() {
    if (getCurrentState() == SpindexerState.FEED && isStalled()) {
      setState(SpindexerState.REVERSE);
    }

    switch (getCurrentState()) {
      case HOLD:
        outputs.indexMotorVoltage = Volts.zero();
        outputs.feedMotorVoltage = Volts.zero();
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
