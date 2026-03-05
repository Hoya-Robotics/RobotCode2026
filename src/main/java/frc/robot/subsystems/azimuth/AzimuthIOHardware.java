package frc.robot.subsystems.azimuth;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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

public class AzimuthIOHardware implements AzimuthIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private final TalonFXSignals signals;

  private PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0.0);

  public AzimuthIOHardware(int motorId, int encoderId) {
    this.motor = new TalonFX(motorId);
    this.encoder = new CANcoder(encoderId);

    // Configure CANcoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(0.0);
    encoder.getConfigurator().apply(encoderConfig);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.yawGains.toSlot0Configs());
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Feedback.FeedbackRemoteSensorID = encoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 0.0;
    config.Feedback.RotorToSensorRatio = 0.0;

    motor.getConfigurator().apply(config);

    this.signals = PhoenixSync.registerTalonFX(motor, TurretConstants.yawGearRatio, 50);
  }

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    inputs.isConnected = signals.isConnected();
    inputs.voltageApplied = signals.getVoltage();
    inputs.velocityRadsPerSec = signals.getVelocityRadsPerSec();
    inputs.positionRads = signals.getPositionRads();
  }

  @Override
  public void setAngle(Angle angle) {
    motor.setControl(request.withPosition(angle));
  }
}
