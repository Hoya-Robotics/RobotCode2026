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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.RobotConfig.SpindexerConstants;
import org.littletonrobotics.junction.Logger;

public class SpindexerIOHardware implements SpindexerIO {
  private final SparkFlex indexMotor;
  private final SparkFlex feedMotor;
  private final RelativeEncoder indexEncoder;
  private final RelativeEncoder feedEncoder;
  private final SparkClosedLoopController indexController;
  private final SparkClosedLoopController feedController;

  public SpindexerIOHardware(int indexId, int feedId) {
    indexMotor = new SparkFlex(indexId, MotorType.kBrushless);
    feedMotor = new SparkFlex(feedId, MotorType.kBrushless);
    indexEncoder = indexMotor.getEncoder();
    feedEncoder = feedMotor.getEncoder();
    indexController = indexMotor.getClosedLoopController();
    feedController = feedMotor.getClosedLoopController();

    SparkFlexConfig indexConfig = new SparkFlexConfig();
    // indexConfig.closedLoop.pid(0.0002, 0.0, 0.0);
    // indexConfig.closedLoop.feedForward.kV(0.0054);
    indexConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(80); // 29
    indexConfig.encoder.positionConversionFactor(SpindexerConstants.indexGearRatio);
    indexConfig.encoder.velocityConversionFactor(SpindexerConstants.indexGearRatio);

    SparkFlexConfig feedConfig = new SparkFlexConfig();
    // feedConfig.closedLoop.pid(0.00002, 0.0, 0.0);
    // feedConfig.closedLoop.feedForward.kV(0.0018); // 0.05
    feedConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(80);
    feedConfig.encoder.positionConversionFactor(SpindexerConstants.feedGearRatio);
    feedConfig.encoder.velocityConversionFactor(SpindexerConstants.feedGearRatio);

    indexMotor.configure(
        indexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    feedMotor.configure(
        feedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SpindexerConstants.indexGains.registerMotor(indexMotor);
    SpindexerConstants.feederGains.registerMotor(feedMotor);
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
    Logger.recordOutput("Spindexer/indexSetpointRPS", outputs.indexMotorVelocity);
    Logger.recordOutput("Spindexer/feedSetpointRPS", outputs.feedVelocity);

    indexController.setSetpoint(outputs.indexMotorVelocity.in(RPM), ControlType.kVelocity);
    feedController.setSetpoint(outputs.feedVelocity.in(RPM), ControlType.kVelocity);
  }
}
