package frc.robot.subsystems.azimuth;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;
import org.littletonrobotics.junction.Logger;

public class AzimuthIOHardware implements AzimuthIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private final TalonFXSignals signals;

  // private PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0.0);
  private PositionVoltage request = new PositionVoltage(0.0);

  public AzimuthIOHardware(int motorId, int encoderId) {
    this.motor = new TalonFX(motorId);
    this.encoder = new CANcoder(encoderId);

    // Configure CANcoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(0.391 + 0.3825);
    encoder.getConfigurator().apply(encoderConfig);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.azimuthGains.toSlot0Configs());
    config.CurrentLimits.withStatorCurrentLimit(20);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Feedback.withRemoteCANcoder(encoder)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1.0)
        .withRotorToSensorRatio(TurretConstants.azimuthGearRatio);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.5)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(-0.5);
    motor.getConfigurator().apply(config);

    this.signals = PhoenixSync.registerTalonFX(motor, 150);
  }

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    inputs.isConnected = signals.isConnected();
    inputs.voltageApplied = signals.getVoltage();
    inputs.current = signals.getCurrent();
    inputs.velocity = signals.getVelocity();
    inputs.position = signals.getPosition();
  }

  @Override
  public void setAngle(Angle angle) {
    Logger.recordOutput("Azimuth/Setpoint", angle);
    motor.setControl(request.withPosition(angle));
  }
}
