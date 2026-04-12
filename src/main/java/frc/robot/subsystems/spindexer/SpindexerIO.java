package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public class SpindexerIOInputs {
    public boolean indexConnected = false;
    public double indexMotorVoltageApplied = 0.0;
    public double indexMotorCurrent = 0.0;
    public double indexMotorVelocityRPS = 0.0;

    public boolean feedConnected = false;
    public double feedMotorVoltageApplied = 0.0;
    public double feedMotorCurrent = 0.0;
    public double feedMotorVelocityRPS = 0.0;

    public boolean rampConnected = false;
    public double rampMotorVoltageApplied = 0.0;
    public double rampMotorCurrent = 0.0;
    public double rampMotorVelocityRPS = 0.0;
  }

  public class SpindexerIOOutputs {
    public double indexSetpointRPS = 0.0;
    public double feedSetpointRPS = 0.0;
    public double rampSetpointRPS = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void applyOutputs(SpindexerIOOutputs outputs) {}
}
