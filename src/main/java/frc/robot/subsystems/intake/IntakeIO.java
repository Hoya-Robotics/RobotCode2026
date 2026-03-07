package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public boolean extendConnected = false;
    public Distance extendPosition = Inches.zero();
    public Voltage extendVoltageApplied = Volts.zero();
    public Current extendCurrent = Amps.zero();
    public AngularVelocity extendVelocity = RadiansPerSecond.zero();

    public boolean intakeConnected = false;
    public Voltage intakeVoltageApplied = Volts.zero();
    public Current intakeCurrent = Amps.zero();
    public AngularVelocity intakeVelocity = RadiansPerSecond.zero();
  }

  public class IntakeIOOutputs {
    public Voltage intakeVoltage = Volts.zero();
    public Voltage extendVoltage = Volts.zero();
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}
