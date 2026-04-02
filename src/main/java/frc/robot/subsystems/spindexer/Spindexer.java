package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

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

    applyState();

    Logger.recordOutput("Spindexer/state", getCurrentState());
    io.applyOutputs(outputs);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case HOLD:
        outputs.indexSetpointRPS = 0.0;
        outputs.feedSetpointRPS = 0.0;
        break;
      case COOLDOWN:
        outputs.indexSetpointRPS = 0.0;
        outputs.feedSetpointRPS = feedSpeed.getAsDouble();
        break;
      case FEED:
        outputs.feedSetpointRPS = feedSpeed.getAsDouble();
        outputs.indexSetpointRPS = indexSpeed.getAsDouble();
        break;
    }
  }
}
