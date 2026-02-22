package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
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
  private final SparkClosedLoopController spinController;
  private final SparkClosedLoopController rampController;

  public SpindexerIOHardware(int spinId, int rampId) {
    spinMotor = new SparkFlex(spinId, MotorType.kBrushless);
    rampMotor = new SparkFlex(rampId, MotorType.kBrushless);
    spinEncoder = spinMotor.getEncoder();
    rampEncoder = rampMotor.getEncoder();
    spinController = spinMotor.getClosedLoopController();
    rampController = rampMotor.getClosedLoopController();

    SparkFlexConfig spinConfig = new SparkFlexConfig();
    spinConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(60);
    /*
    spinConfig.closedLoop
      .p(0.0)
      .i(0.0)
      .d(0.0)
      .outputRange(-1.0, 1.0);
    spinConfig.closedLoop.feedForward
      .kV(0.0).kS(0.0);
    */

    SparkFlexConfig rampConfig = new SparkFlexConfig();
    rampConfig.idleMode(IdleMode.kBrake).inverted(false).smartCurrentLimit(60);
    /*
    rampConfig.closedLoop
      .p(0.0)
      .i(0.0)
      .d(0.0)
      .outputRange(-1.0, 1.0);
    rampConfig.closedLoop.feedForward
      .kV(0.0).kS(0.0);
    */

    spinMotor.configure(
        spinConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rampMotor.configure(
        rampConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.spinMotorVoltsApplied = spinMotor.getBusVoltage();
    inputs.spinMotorVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(spinEncoder.getVelocity() / (3.0 / 1.0));

    inputs.rampMotorVoltsApplied = rampMotor.getBusVoltage();
    inputs.rampMotorVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rampEncoder.getVelocity());
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    switch (outputs.mode) {
      case VOLTAGE:
        spinMotor.setVoltage(outputs.spinMotorVoltageRequested);
        rampMotor.setVoltage(outputs.rampMotorVoltageRequested);
        break;
      case CLOSED_LOOP:
        spinController.setSetpoint(
            outputs.spinMotorSpeedRequested.in(RadiansPerSecond), ControlType.kVelocity);
        rampController.setSetpoint(
            outputs.rampMotorSpeedRequested.in(MetersPerSecond), ControlType.kVelocity);
        break;
    }
  }
}
