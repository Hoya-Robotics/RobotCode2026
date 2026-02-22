package frc.robot.subsystems.spindexer;

import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum SpindexerState {
  HOLD,
  FEED,
}

// TODO: probably move this into superstructure
public class Spindexer extends StateSubsystem<SpindexerState> {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private SpindexerIOOutputs outputs = new SpindexerIOOutputs();

  public Spindexer(SpindexerIO io) {
    this.io = io;
    setState(SpindexerState.FEED);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Spindexer", inputs);

    applyState();
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case HOLD:
        break;
      case FEED:
        outputs.spinMotorVoltageRequested = 3.0;
        outputs.rampMotorVoltageRequested = 3.0;
        break;
    }
  }
}
