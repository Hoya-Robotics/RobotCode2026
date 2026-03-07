package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public class SpindexerIOInputs {
    public boolean indexConnected = false;
    public Voltage indexMotorVoltageApplied = Volts.zero();
    public Current indexMotorCurrent = Amps.zero();
    public AngularVelocity indexMotorVelocity = RadiansPerSecond.zero();

    public boolean feedConnected = false;
    public Voltage feedMotorVoltageApplied = Volts.zero();
    public Current feedMotorCurrent = Amps.zero();
    public AngularVelocity feedMotorVelocity = RadiansPerSecond.zero();
  }

  public class SpindexerIOOutputs {
    public Voltage indexMotorVoltage = Volts.zero();
    public Voltage feedMotorVoltage = Volts.zero();
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void applyOutputs(SpindexerIOOutputs outputs) {}
}
