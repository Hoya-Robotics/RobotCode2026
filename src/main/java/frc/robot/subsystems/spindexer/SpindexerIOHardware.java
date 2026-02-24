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
import edu.wpi.first.math.util.Units;

public class SpindexerIOHardware implements SpindexerIO {
  private final SparkFlex spinMotor;
  private final SparkFlex rampMotor;
  private final RelativeEncoder spinEncoder;
  private final RelativeEncoder rampEncoder;

  public SpindexerIOHardware(int spinId, int rampId) {
    spinMotor = new SparkFlex(spinId, MotorType.kBrushless);
    rampMotor = new SparkFlex(rampId, MotorType.kBrushless);
    spinEncoder = spinMotor.getEncoder();
    rampEncoder = rampMotor.getEncoder();

    SparkFlexConfig spinConfig = new SparkFlexConfig();
    spinConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);

    SparkFlexConfig rampConfig = new SparkFlexConfig();
    rampConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);

    spinMotor.configure(
        spinConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rampMotor.configure(
        rampConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.spinMotorVoltsApplied = spinMotor.getAppliedOutput() * spinMotor.getBusVoltage();
    inputs.spinMotorVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(spinEncoder.getVelocity() / 3.0);
		inputs.spinConnected = spinMotor.getLastError() == REVLibError.kOk;

    inputs.rampMotorVoltsApplied = rampMotor.getAppliedOutput() * rampMotor.getBusVoltage();
    inputs.rampMotorVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rampEncoder.getVelocity());
		inputs.rampConnected = rampMotor.getLastError() == REVLibError.kOk;
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
			spinMotor.setVoltage(outputs.spinMotorVoltageRequested);
			rampMotor.setVoltage(outputs.rampMotorVoltageRequested);
  }
}
