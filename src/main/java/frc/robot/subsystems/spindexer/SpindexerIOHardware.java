package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.RobotConfig.SpindexerConstants;
import org.littletonrobotics.junction.Logger;

public class SpindexerIOHardware implements SpindexerIO {
  private final SparkFlex indexMotor;
  private final SparkFlex feedMotor;
  private final RelativeEncoder indexEncoder;
  private final RelativeEncoder feedEncoder;

  public SpindexerIOHardware(int indexId, int feedId) {
    indexMotor = new SparkFlex(indexId, MotorType.kBrushless);
    feedMotor = new SparkFlex(feedId, MotorType.kBrushless);
    indexEncoder = indexMotor.getEncoder();
    feedEncoder = feedMotor.getEncoder();

    SparkFlexConfig indexConfig = new SparkFlexConfig();
    indexConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(29);
    indexConfig.encoder.positionConversionFactor(SpindexerConstants.indexGearRatio);

    SparkFlexConfig feedConfig = new SparkFlexConfig();
    feedConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);
    feedConfig.encoder.positionConversionFactor(SpindexerConstants.feedGearRatio);

    indexMotor.configure(
        indexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    feedMotor.configure(
        feedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.indexMotorVoltageApplied =
        Volts.of(indexMotor.getAppliedOutput() * indexMotor.getBusVoltage());
    inputs.indexMotorCurrent = Amps.of(indexMotor.getOutputCurrent());
    inputs.indexMotorVelocity = RPM.of(indexEncoder.getVelocity());
    inputs.indexConnected = indexMotor.getLastError() == REVLibError.kOk;

    inputs.feedMotorVoltageApplied =
        Volts.of(feedMotor.getAppliedOutput() * feedMotor.getBusVoltage());
    inputs.feedMotorCurrent = Amps.of(feedMotor.getOutputCurrent());
    inputs.feedMotorVelocity = RPM.of(feedEncoder.getVelocity());
    inputs.feedConnected = feedMotor.getLastError() == REVLibError.kOk;
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    Logger.recordOutput("Spindexer/indexSetpoint", outputs.indexMotorVoltage);
    Logger.recordOutput("Spindexer/feedSetpoint", outputs.feedMotorVoltage);

    indexMotor.setVoltage(outputs.indexMotorVoltage.in(Volts));
    feedMotor.setVoltage(outputs.feedMotorVoltage.in(Volts));
  }
}
