package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public boolean extendConnected = false;
		public double extendPositionRads = 0.0;
    public double extendVoltageApplied = 0.0;
    public double extendVelocityRadsPerSec = 0.0;

    public boolean spinConnected = false;
    public double spinVoltageApplied = 0.0;
    public double spinVelocityRadsPerSec = 0.0;
  }

  public class IntakeIOOutputs {
    public double spinVoltage = 0.0;
    public double extendMeters = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}
