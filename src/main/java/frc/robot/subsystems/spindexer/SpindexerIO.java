package frc.robot.subsystems.spindexer;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public class SpindexerIOInputs {
    public boolean spinConnected = false;
    public double spinMotorVoltsApplied = 0.0;
    public double spinMotorVelocityRadsPerSec = 0.0;

    public boolean rampConnected = false;
    public double rampMotorVoltsApplied = 0.0;
    public double rampMotorVelocityRadsPerSec = 0.0;
  }

  public class SpindexerIOOutputs {
    public double spinMotorVoltageRequested = 0.0;
    public double rampMotorVoltageRequested = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void applyOutputs(SpindexerIOOutputs outputs) {}
}
