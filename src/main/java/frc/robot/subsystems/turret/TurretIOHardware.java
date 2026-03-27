package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
  // private final PositionVoltage azimuthRequest = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC azimuthRequest = new PositionTorqueCurrentFOC(0.0);

  private final TalonFX hoodMotor;
  private final TalonFXSignals hoodSignals;
  private final PositionVoltage hoodRequest = new PositionVoltage(0.0);

  private final TalonFX shooterMotor;
  private final TalonFXSignals shooterSignals;
  private final VelocityTorqueCurrentFOC shooterRequest = new VelocityTorqueCurrentFOC(0.0);

  public TurretIOHardware(int azimuthId, int azimuthEncoderId, int hoodId, int shooterId) {
    azimuthMotor = new TalonFX(azimuthId);
    azimuthEncoder = new CANcoder(azimuthEncoderId);
    hoodMotor = new TalonFX(hoodId);
    shooterMotor = new TalonFX(shooterId);

    configureAzimuth();
    configureHood();
    configureShooter();

    hoodMotor.setPosition(0.0);

    azimuthSignals = PhoenixSync.registerTalonFX(azimuthMotor, 150);
    hoodSignals = PhoenixSync.registerTalonFX(hoodMotor, 150);
    shooterSignals = PhoenixSync.registerTalonFX(shooterMotor, 150);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.azimuthState = MotorState.fromTalonSignals(azimuthSignals);
    inputs.hoodState = MotorState.fromTalonSignals(hoodSignals);
    inputs.shooterState = MotorState.fromTalonSignals(shooterSignals);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    azimuthMotor.setControl(azimuthRequest.withPosition(outputs.azimuthSetpoint));
    // .withVelocity(outputs.azimuthVelocitySetpoint));
    hoodMotor.setControl(hoodRequest.withPosition(outputs.hoodSetpoint));
    shooterMotor.setControl(shooterRequest.withVelocity(outputs.shooterSetpoint));
  }

  private void configureAzimuth() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.withMagnetOffset(0.1557);
    azimuthEncoder.getConfigurator().apply(encoderConfig);

    var config = new TalonFXConfiguration();

    config.withSlot0(
        new Slot0Configs()
            .withKS(12.0)
            .withKP(90)
            .withKA(2.13)
            .withKD(7.5)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
    // config.withSlot0(TurretConstants.azimuthGains.toSlot0Configs());
    config.CurrentLimits.withStatorCurrentLimit(20);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    config.Feedback.withFusedCANcoder(azimuthEncoder)
        .withSensorToMechanismRatio(1.0)
        .withRotorToSensorRatio(TurretConstants.azimuthGearRatio);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(TurretConstants.maxAzimuthAngle.in(Rotations))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(TurretConstants.minAzimuthAngle.in(Rotations));
    azimuthMotor.getConfigurator().apply(config);
  }

  private void configureHood() {
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.hoodGains.toSlot0Configs());
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    config.Feedback.withSensorToMechanismRatio(TurretConstants.hoodGearRatio);
    config.CurrentLimits.withStatorCurrentLimit(20);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.107)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0.0);
    hoodMotor.getConfigurator().apply(config);
  }

  private void configureShooter() {
    var config = new TalonFXConfiguration();
    config.withSlot0(new Slot0Configs().withKS(12.3).withKV(0.36).withKP(30));
    config.Feedback.withSensorToMechanismRatio(TurretConstants.launcherGearRatio);
    config.CurrentLimits.withStatorCurrentLimit(100);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    shooterMotor.getConfigurator().apply(config);
  }
}
