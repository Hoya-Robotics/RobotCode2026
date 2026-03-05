package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConfig.SpindexerConstants;
import org.littletonrobotics.junction.Logger;

public class SpindexerIOHardware implements SpindexerIO {
  private final SparkFlex indexMotor;
  private final SparkFlex feedMotor;
  private final RelativeEncoder indexEncoder;
  private final RelativeEncoder feedEncoder;

  public SpindexerIOHardware(int spinId, int rampId) {
    indexMotor = new SparkFlex(spinId, MotorType.kBrushless);
    feedMotor = new SparkFlex(rampId, MotorType.kBrushless);
    indexEncoder = indexMotor.getEncoder();
    feedEncoder = feedMotor.getEncoder();

    SparkFlexConfig spinConfig = new SparkFlexConfig();
    spinConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);

    SparkFlexConfig rampConfig = new SparkFlexConfig();
    rampConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);

    indexMotor.configure(
        spinConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    feedMotor.configure(
        rampConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.indexMotorVoltsApplied = indexMotor.getAppliedOutput() * indexMotor.getBusVoltage();
    inputs.indexMotorVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(indexEncoder.getVelocity())
            / SpindexerConstants.indexGearRatio;
    inputs.indexConnected = indexMotor.getLastError() == REVLibError.kOk;

    inputs.feedMotorVoltsApplied = feedMotor.getAppliedOutput() * feedMotor.getBusVoltage();
    inputs.feedMotorVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(feedEncoder.getVelocity())
            / SpindexerConstants.feedGearRatio;
    inputs.feedConnected = feedMotor.getLastError() == REVLibError.kOk;
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    Logger.recordOutput("Spindexer/indexSetpoint", outputs.indexMotorVoltageRequested);
    Logger.recordOutput("Spindexer/feedSetpoint", outputs.feedMotorVoltageRequested);

    indexMotor.setVoltage(outputs.indexMotorVoltageRequested);
    feedMotor.setVoltage(outputs.feedMotorVoltageRequested);
  }
}
