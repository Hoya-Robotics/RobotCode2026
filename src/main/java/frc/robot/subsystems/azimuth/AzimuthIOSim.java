// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.azimuth;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AzimuthIOSim implements AzimuthIO {
  private DCMotor gearbox = DCMotor.getKrakenX60(1);
  private DCMotorSim turretSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.07867078, TurretConstants.azimuthGearRatio),
          gearbox);

  private final TalonFX motor;
  private final CANcoder encoder;
  private final TalonFXSignals signals;

  private PositionVoltage request = new PositionVoltage(0.0);

  public AzimuthIOSim(int motorId, int encoderId) {
    this.motor = new TalonFX(motorId);
    this.encoder = new CANcoder(encoderId);

    // Configure CANcoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // encoderConfig.MagnetSensor.withMagnetOffset(0.391 + 0.3825);
    encoder.getConfigurator().apply(encoderConfig);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.azimuthGains.toSlot0Configs());
    config.CurrentLimits.withStatorCurrentLimit(20);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    // .withNeutralMode(NeutralModeValue.Brake);
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

    turretSim.setState(0.0, 0.0);

    var CANcoderSim = encoder.getSimState();
    CANcoderSim.Orientation = ChassisReference.Clockwise_Positive;
    var talonFXSim = motor.getSimState();
    talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
    talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void updateInputs(AzimuthIOInputs inputs) {
    var CANcoderSim = encoder.getSimState();
    var talonFXSim = motor.getSimState();

    CANcoderSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    talonFXSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

    var motorVoltage = talonFXSim.getMotorVoltageMeasure();
    turretSim.setInputVoltage(motorVoltage.in(Volts));
    turretSim.update(0.020);

    CANcoderSim.setRawPosition(turretSim.getAngularPosition());
    CANcoderSim.setVelocity(turretSim.getAngularVelocity());
    talonFXSim.setRawRotorPosition(
        turretSim.getAngularPosition().times(TurretConstants.azimuthGearRatio));
    talonFXSim.setRotorVelocity(
        turretSim.getAngularVelocity().times(TurretConstants.azimuthGearRatio));

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
