package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MotorState;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;

public class TurretIOHardware implements TurretIO {
  private final TalonFX azimuthMotor;
  private final CANcoder azimuthEncoder;
  private final TalonFXSignals azimuthSignals;
  private final PositionTorqueCurrentFOC azimuthTrackRequest = new PositionTorqueCurrentFOC(0.0);

  private LoggedTunableNumber flywheelTuningSetpoint =
      new LoggedTunableNumber("Turret/Flywheel/TuningSetpoint", 20.0);

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

    azimuthSignals = PhoenixSync.registerTalonFX(azimuthMotor, 150);
    hoodSignals = PhoenixSync.registerTalonFX(hoodMotor, 150);
    leftFlywheelSignals = PhoenixSync.registerTalonFX(leftFlywheelMotor, 150);
    rightFlywheelSignals = PhoenixSync.registerTalonFX(rightFlywheelMotor, 150);

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
    /*
    azimuthMotor.setControl(
        azimuthTrackRequest
            .withPosition(outputs.azimuthSetpointRots)
            .withFeedForward(outputs.azimuthFeedforward));*/
    hoodMotor.setControl(hoodRequest.withPosition(outputs.hoodSetpointRots));
    // leftFlywheelMotor.setControl(flywheelRequest.withVelocity(outputs.flywheelRPS));
    // rightFlywheelMotor.setControl(flywheelRequest.withVelocity(outputs.flywheelRPS));
    leftFlywheelMotor.setControl(
        flywheelRequest.withVelocity(flywheelTuningSetpoint.getAsDouble()));
    rightFlywheelMotor.setControl(
        flywheelRequest.withVelocity(flywheelTuningSetpoint.getAsDouble()));
  }

  private void configureAzimuth() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(0.1557);
    azimuthEncoder.getConfigurator().apply(encoderConfig);

    var config = new TalonFXConfiguration();
    config.withSlot0(
        new Slot0Configs().withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
    config.CurrentLimits.withStatorCurrentLimit(60);
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
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    config.Feedback.withSensorToMechanismRatio(TurretConstants.hoodGearRatio);
    config.CurrentLimits.withStatorCurrentLimit(60);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.107)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0.0);
    for (int i = 0; i < 5; ++i) {
      if (hoodMotor.getConfigurator().apply(config).isOK()) break;
    }
  }

  private void configureFlywheel(TalonFX motor, boolean invert) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.withStatorCurrentLimit(60);
    config.MotorOutput.withInverted(invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    for (int i = 0; i < 5; ++i) {
      if (motor.getConfigurator().apply(config).isOK()) break;
    }
  }
}
