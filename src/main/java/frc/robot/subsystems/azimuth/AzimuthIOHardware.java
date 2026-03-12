package frc.robot.subsystems.azimuth;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
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
    encoderConfig.MagnetSensor.withMagnetOffset(0.89);
    encoder.getConfigurator().apply(encoderConfig);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.azimuthGains.toSlot0Configs());
    config.CurrentLimits.withStatorCurrentLimit(20);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    // .withNeutralMode(NeutralModeValue.Brake);
		// config.ClosedLoopGeneral.withContinuousWrap(true);
    config.Feedback.withRemoteCANcoder(encoder)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        //.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1.0)
        .withRotorToSensorRatio(TurretConstants.azimuthGearRatio);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(TurretConstants.maxAzimuthAngle.in(Rotations))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(TurretConstants.minAzimuthAngle.in(Rotations));
    motor.getConfigurator().apply(config);
		// motor.setPosition(encoder.getPosition().getValue());

    this.signals = PhoenixSync.registerTalonFX(motor, 150);
  }

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    inputs.isConnected = signals.isConnected();
    inputs.voltageApplied = signals.getVoltage();
    inputs.current = signals.getCurrent();
    inputs.velocity = signals.getVelocity();
    inputs.position = signals.getPosition();
		Logger.recordOutput("Azimuth/absolutePosition", encoder.getPosition().getValueAsDouble());
  }

  @Override
  public void setAngle(Angle angle) {
		double targetRotations = angle.in(Rotations);
		double currentRotations = signals.getPosition().in(Rotations);
		double diff = targetRotations - currentRotations;
		diff = MathUtil.inputModulus(diff, -0.5, 0.5);
		double closestTarget = currentRotations + diff;
		if (closestTarget > TurretConstants.maxAzimuthAngle.in(Rotations)) {
			closestTarget -= 1;
		}
		if (closestTarget < TurretConstants.minAzimuthAngle.in(Rotations)) {
			closestTarget += 1;
		}

		Angle finalAngle = Rotations.of(closestTarget);
    Logger.recordOutput("Azimuth/Setpoint", finalAngle);
    motor.setControl(request.withPosition(finalAngle));
  }
}
