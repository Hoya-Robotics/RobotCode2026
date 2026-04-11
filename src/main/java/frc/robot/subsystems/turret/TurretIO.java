package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import frc.robot.util.MotorState;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public class TurretIOInputs {
    public MotorState azimuthState = MotorState.kZero;
    public MotorState hoodState = MotorState.kZero;
    public MotorState leftFlywheelState = MotorState.kZero;
    public MotorState rightFlywheelState = MotorState.kZero;
  }

  public class TurretIOOutputs {
    public double azimuthSetpointRots = 0.0;
    public double azimuthFFRotsPerSec = 0.0;
    public double hoodSetpointRots = 0.0;
    public double flywheelRPS = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void applyOutputs(TurretIOOutputs outputs) {}
}
