package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public boolean extendConnected = false;
    public Distance extendPosition = Inches.zero();
    public Voltage extendVoltageApplied = Volts.zero();
    public Current extendCurrent = Amps.zero();
    public LinearVelocity extendVelocity = MetersPerSecond.zero();

    public boolean intakeConnected = false;
    public Voltage intakeVoltageApplied = Volts.zero();
    public Current intakeCurrent = Amps.zero();
    public AngularVelocity intakeVelocity = RadiansPerSecond.zero();
  }

  public enum ExtendControlType {
    POSITION,
    VOLTAGE
  }

  public class IntakeIOOutputs {
    public double intakeVelocityRPM = 0.0;
    public ExtendControlType extendControlType = ExtendControlType.POSITION;
    public double extendSetpointInches = 0.0;
    public double extendVoltage = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}
