package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.MotorState;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public class TurretIOInputs {
    public MotorState azimuthState = MotorState.kZero;
    public MotorState hoodState = MotorState.kZero;
    public MotorState shooterState = MotorState.kZero;
  }

  public class TurretIOOutputs {
    public Angle azimuthSetpoint = Rotations.of(0.0);
    public Angle hoodSetpoint = Rotations.of(0.0);
    public AngularVelocity shooterSetpoint = RotationsPerSecond.of(0.0);
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void applyOutputs(TurretIOOutputs outputs) {}
}
