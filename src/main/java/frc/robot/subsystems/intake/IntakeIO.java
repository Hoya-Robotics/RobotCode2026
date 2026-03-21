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

  public class IntakeIOOutputs {
    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0.0);
    public Distance extensionDistance = Meters.of(0.0);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}
