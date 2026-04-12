package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotConfig.SpindexerConstants;

public class SpindexerIOHardware implements SpindexerIO {
  private final SparkFlex indexMotor;
  private final SparkFlex feedMotor;
  private final SparkMax rampMotor;
  private final RelativeEncoder indexEncoder;
  private final RelativeEncoder feedEncoder;
  private final RelativeEncoder rampEncoder;
  private final SparkClosedLoopController indexController;
  private final SparkClosedLoopController feedController;
  private final SparkClosedLoopController rampController;

  public SpindexerIOHardware(int indexId, int feedId, int rampId) {
    indexMotor = new SparkFlex(indexId, MotorType.kBrushless);
    feedMotor = new SparkFlex(feedId, MotorType.kBrushless);
    rampMotor = new SparkMax(rampId, MotorType.kBrushless);

    indexEncoder = indexMotor.getEncoder();
    feedEncoder = feedMotor.getEncoder();
    rampEncoder = rampMotor.getEncoder();
    indexController = indexMotor.getClosedLoopController();
    feedController = feedMotor.getClosedLoopController();
    rampController = rampMotor.getClosedLoopController();

    SparkMaxConfig rampConfig = new SparkMaxConfig();
    rampConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rampConfig.encoder.positionConversionFactor(SpindexerConstants.rampGearRatio);
    rampConfig.encoder.velocityConversionFactor(SpindexerConstants.rampGearRatio);

    SparkFlexConfig indexConfig = new SparkFlexConfig();
    indexConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(50); // 29
    indexConfig.encoder.positionConversionFactor(SpindexerConstants.indexGearRatio);
    indexConfig.encoder.velocityConversionFactor(SpindexerConstants.indexGearRatio);

    SparkFlexConfig feedConfig = new SparkFlexConfig();
    feedConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(50);
    feedConfig.encoder.positionConversionFactor(SpindexerConstants.feedGearRatio);
    feedConfig.encoder.velocityConversionFactor(SpindexerConstants.feedGearRatio);

    indexMotor.configure(
        indexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rampMotor.configure(
        rampConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    feedMotor.configure(
        feedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SpindexerConstants.indexGains.registerMotor(indexMotor);
    SpindexerConstants.rampGains.registerMotor(rampMotor);
    SpindexerConstants.feederGains.registerMotor(feedMotor);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.indexMotorVoltageApplied =
        Volts.of(indexMotor.getAppliedOutput() * indexMotor.getBusVoltage());
    inputs.indexMotorCurrent = Amps.of(indexMotor.getOutputCurrent());
    inputs.indexMotorVelocity = RotationsPerSecond.of(indexEncoder.getVelocity() / 60.0);
    inputs.indexConnected = indexMotor.getLastError() == REVLibError.kOk;

    inputs.feedMotorVoltageApplied =
        Volts.of(feedMotor.getAppliedOutput() * feedMotor.getBusVoltage());
    inputs.feedMotorCurrent = Amps.of(feedMotor.getOutputCurrent());
    inputs.feedMotorVelocity = RotationsPerSecond.of(feedEncoder.getVelocity() / 60.0);
    inputs.feedConnected = feedMotor.getLastError() == REVLibError.kOk;

    inputs.rampMotorVoltageApplied =
        Volts.of(rampMotor.getAppliedOutput() * rampMotor.getBusVoltage());
    inputs.rampMotorCurrent = Amps.of(rampMotor.getOutputCurrent());
    inputs.rampMotorVelocity = RotationsPerSecond.of(rampEncoder.getVelocity() / 60.0);
    inputs.rampConnected = rampMotor.getLastError() == REVLibError.kOk;
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    indexController.setSetpoint(outputs.indexSetpointRPS * 60, ControlType.kVelocity);
    feedController.setSetpoint(outputs.feedSetpointRPS * 60, ControlType.kVelocity);
    rampController.setSetpoint(outputs.rampSetpointRPS * 60, ControlType.kVelocity);
  }
}
