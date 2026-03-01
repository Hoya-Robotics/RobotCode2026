package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public class TurretIOInputs {
    public boolean yawConnected = false;
    public double yawVoltageApplied = 0.0;
		public double yawPositionRad = 0.0;
    public double yawVelocityRadsPerSec = 0.0;

    public boolean hoodConnected = false;
    public double hoodVoltageApplied = 0.0;
		public double hoodPositionRad = 0.0;
    public double hoodVelocityRadsPerSec = 0.0;

    public boolean shootConnected = false;
    public double shootVoltageApplied = 0.0;
		public double shootPositionRad = 0.0;
    public double shootVelocityRadsPerSec = 0.0;
  }

  public class TurretIOOutputs {
    public double yawAngle;
    public double hoodAngle;
    public double flywheelSpeed;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void applyOutputs(TurretIOOutputs outputs) {}
}
