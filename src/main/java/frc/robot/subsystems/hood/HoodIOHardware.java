package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;
import org.littletonrobotics.junction.Logger;

public class HoodIOHardware implements HoodIO {
  private final TalonFX motor;
  private final TalonFXSignals signals;

  // private PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0.0);
  private PositionVoltage request = new PositionVoltage(0.0);

  public HoodIOHardware(int motorId) {
    this.motor = new TalonFX(motorId);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.hoodGains.toSlot0Configs());
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Feedback.withSensorToMechanismRatio(TurretConstants.hoodGearRatio);
    config.CurrentLimits.withStatorCurrentLimit(20);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.107)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0.0);

    motor.getConfigurator().apply(config);
    motor.setPosition(0.0);

    this.signals = PhoenixSync.registerTalonFX(motor, 150);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.isConnected = signals.isConnected();
    inputs.voltageApplied = signals.getVoltage();
    inputs.current = signals.getCurrent();
    inputs.velocity = signals.getVelocity();
    inputs.position = signals.getPosition();
  }

  public void setAngle(Angle angle) {
    Logger.recordOutput("Hood/setpoint", angle);
    motor.setControl(request.withPosition(angle));
  }
}
