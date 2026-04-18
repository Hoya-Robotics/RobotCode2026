package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.MotorState;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;

public class TurretIOHardware implements TurretIO {
  private final TalonFX azimuthMotor;
  private final CANcoder azimuthEncoder;
  private final TalonFXSignals azimuthSignals;
  private final PositionVoltage azimuthRequest = new PositionVoltage(0.0);

  private final TalonFX hoodMotor;
  private final TalonFXSignals hoodSignals;
  private final PositionVoltage hoodRequest = new PositionVoltage(0.0);

  private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;
  private final TalonFXSignals leftFlywheelSignals;
  private final TalonFXSignals rightFlywheelSignals;
  private final VelocityVoltage flywheelRequest = new VelocityVoltage(0.0);

  public TurretIOHardware(
      int azimuthId, int azimuthEncoderId, int hoodId, int LFlywheelId, int RFlywheelId) {
    azimuthMotor = new TalonFX(azimuthId);
    azimuthEncoder = new CANcoder(azimuthEncoderId);
    hoodMotor = new TalonFX(hoodId);
    leftFlywheelMotor = new TalonFX(LFlywheelId);
    rightFlywheelMotor = new TalonFX(RFlywheelId);

    configureAzimuth();
    configureHood();
    configureFlywheel(leftFlywheelMotor, false);
    configureFlywheel(rightFlywheelMotor, true);

    TurretConstants.azimuthGains.registerMotor(azimuthMotor);
    TurretConstants.hoodGains.registerMotor(hoodMotor);
    TurretConstants.leftFlywheelGains.registerMotor(leftFlywheelMotor);
    TurretConstants.rightFlywheelGains.registerMotor(rightFlywheelMotor);

    azimuthSignals = PhoenixSync.registerTalonFX(azimuthMotor, 250);
    hoodSignals = PhoenixSync.registerTalonFX(hoodMotor, 100);
    leftFlywheelSignals = PhoenixSync.registerTalonFX(leftFlywheelMotor, 100);
    rightFlywheelSignals = PhoenixSync.registerTalonFX(rightFlywheelMotor, 100);

    hoodMotor.setPosition(0.0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.azimuthState = MotorState.fromTalonSignals(azimuthSignals);
    inputs.hoodState = MotorState.fromTalonSignals(hoodSignals);
    inputs.rightFlywheelState = MotorState.fromTalonSignals(rightFlywheelSignals);
    inputs.leftFlywheelState = MotorState.fromTalonSignals(leftFlywheelSignals);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    azimuthMotor.setControl(
        azimuthRequest
            .withPosition(outputs.azimuthSetpointRots)
            .withFeedForward(outputs.azimuthFeedforward));
    hoodMotor.setControl(hoodRequest.withPosition(outputs.hoodSetpointRots));
    leftFlywheelMotor.setControl(flywheelRequest.withVelocity(outputs.flywheelRPS));
    rightFlywheelMotor.setControl(flywheelRequest.withVelocity(outputs.flywheelRPS));
  }

  private void configureAzimuth() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(0.574956); // -0.425244, 0.1557
    azimuthEncoder.getConfigurator().apply(encoderConfig);

    var config = new TalonFXConfiguration();
    config.withSlot0(
        new Slot0Configs().withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
    config.CurrentLimits.withStatorCurrentLimit(30);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    config.Feedback.withFusedCANcoder(azimuthEncoder)
        .withSensorToMechanismRatio(1.0)
        .withRotorToSensorRatio(TurretConstants.azimuthGearRatio);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(TurretConstants.maxAzimuthAngle.in(Rotations))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(TurretConstants.minAzimuthAngle.in(Rotations));

    for (int i = 0; i < 5; ++i) {
      if (azimuthMotor.getConfigurator().apply(config).isOK()) break;
    }
  }

  private void configureHood() {
    var config = new TalonFXConfiguration();
    config.withSlot0(
        new Slot0Configs().withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    config.Feedback.withSensorToMechanismRatio(TurretConstants.hoodGearRatio);
    config.CurrentLimits.withStatorCurrentLimit(30);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.078)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0.02);
    for (int i = 0; i < 5; ++i) {
      if (hoodMotor.getConfigurator().apply(config).isOK()) break;
    }
  }

  private void configureFlywheel(TalonFX motor, boolean invert) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.withStatorCurrentLimit(80);
    config.MotorOutput.withInverted(
            invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    for (int i = 0; i < 5; ++i) {
      if (motor.getConfigurator().apply(config).isOK()) break;
    }
  }
}
