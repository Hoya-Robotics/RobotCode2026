package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;

import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeIOHardware implements IntakeIO {
  private final TalonFX extendMotor;
	private final TalonFXSignals extendSignals;

  private final SparkFlex spinMotor;
	private final RelativeEncoder spinEncoder;

  private PositionTorqueCurrentFOC extendRequest = new PositionTorqueCurrentFOC(0.0);

  public IntakeIOHardware(TalonFX extendMotor, int spinId) {
    this.extendMotor = extendMotor;
    this.spinMotor = new SparkFlex(spinId, MotorType.kBrushless);
		spinEncoder = spinMotor.getEncoder();

    var spinConfig = new SparkFlexConfig();
    spinConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(60);
    spinMotor.configure(
        spinConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    var extendConfig = new TalonFXConfiguration();
    // extendConfig.Slot0.withKP(i)
    extendMotor.getConfigurator().apply(extendConfig);

		PhoenixSync.registerTalonFX(extendMotor, 50);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
		inputs.extendConnected = extendSignals.isConnected();
		inputs.extendPositionRads = extendSignals.getPositionRads();
		inputs.extendVelocityRadsPerSec = extendSignals.getVelocityRadsPerSec();
		inputs.extendVoltageApplied = extendSignals.getVoltage();

		inputs.spinConnected = spinMotor.getLastError() == REVLibError.kOk;
		inputs.spinVoltageApplied = spinMotor.getAppliedOutput() * spinMotor.getBusVoltage();
		inputs.spinVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(spinEncoder.getVelocity());
	}

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    // Convert extend meters to motor then use position control
    spinMotor.setVoltage(outputs.spinVoltage);
  }
}
