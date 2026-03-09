package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum SpindexerState {
  HOLD,
  FEED,
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
    Logger.processInputs("Superstructure/Spindexer", inputs);

    applyState();
    io.applyOutputs(outputs);
  }

  public void hold() {
    setState(SpindexerState.HOLD);
  }

  public void feed() {
    setState(SpindexerState.FEED);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case HOLD:
        outputs.indexMotorVoltage = Volts.zero();
        outputs.feedMotorVoltage = Volts.zero();
        break;
      case FEED:
        outputs.indexMotorVoltage = Volts.of(4.5);
        outputs.feedMotorVoltage = Volts.of(7.5);
        break;
    }
  }
}
