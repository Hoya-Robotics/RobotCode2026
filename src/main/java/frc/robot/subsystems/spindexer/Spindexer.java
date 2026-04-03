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

  private LoggedTunableNumber indexSpeed = new LoggedTunableNumber("Spindexer/indexSpeed", 8.0);
  private LoggedTunableNumber feedSpeed = new LoggedTunableNumber("Spindexer/feedSpeed", 18.0);

  private double latestFeedSpeed = 18.0;
  private double latestIndexSpeed = 8.0;

  public Spindexer(SpindexerIO io) {
    this.io = io;
    setState(SpindexerState.HOLD);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
    Logger.recordOutput("Spindexer/state", getCurrentState());

    Robot.batteryLogger.reportCurrentUsage("Spindexer/Indexer", inputs.indexMotorCurrent.in(Amps));
    Robot.batteryLogger.reportCurrentUsage("Spindexer/Feeder", inputs.feedMotorCurrent.in(Amps));

    if (feedSpeed.hasChanged(feedSpeed.hashCode())) {
      latestFeedSpeed = feedSpeed.getAsDouble();
    }

    if (indexSpeed.hasChanged(indexSpeed.hashCode())) {
      latestIndexSpeed = indexSpeed.getAsDouble();
    }

    applyState();

    Logger.recordOutput("Spindexer/Setpoints/indexRPS", outputs.indexSetpointRPS);
    Logger.recordOutput("Spindexer/Setpoints/feedRPS", outputs.feedSetpointRPS);

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
        outputs.feedSetpointRPS = latestFeedSpeed;
        outputs.indexSetpointRPS = 0.0;
        break;
      case FEED:
        outputs.feedSetpointRPS = latestFeedSpeed;
        outputs.indexSetpointRPS = latestIndexSpeed;
        break;
    }
  }
}
