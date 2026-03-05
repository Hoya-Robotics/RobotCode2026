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
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;
import org.littletonrobotics.junction.Logger;

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
    extendConfig.withSlot0(IntakeConstants.extendGains.toSlot0Configs());
    extendMotor.getConfigurator().apply(extendConfig);

    extendSignals = PhoenixSync.registerTalonFX(extendMotor, IntakeConstants.extendGearRatio, 50);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extendConnected = extendSignals.isConnected();
    inputs.extendPositionRads = extendSignals.getPositionRads();
    inputs.extendVelocityRadsPerSec = extendSignals.getVelocityRadsPerSec();
    inputs.extendVoltageApplied = extendSignals.getVoltage();

    inputs.spinConnected = spinMotor.getLastError() == REVLibError.kOk;
    inputs.spinVoltageApplied = spinMotor.getAppliedOutput() * spinMotor.getBusVoltage();
    inputs.spinVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(spinEncoder.getVelocity())
            / IntakeConstants.intakeGearRatio;
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    Logger.recordOutput("Intake/extensionSetpoint", outputs.extendMeters);
    Logger.recordOutput("Intake/spinSetpoint", outputs.spinVoltage);

    double extendRadians =
        outputs.extendMeters / IntakeConstants.extensionRadius * IntakeConstants.extendGearRatio;
    extendMotor.setControl(extendRequest.withPosition(extendRadians));
    spinMotor.setVoltage(outputs.spinVoltage);
  }
}
