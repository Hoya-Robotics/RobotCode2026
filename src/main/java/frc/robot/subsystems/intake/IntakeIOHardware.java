package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;
import org.littletonrobotics.junction.Logger;

public class IntakeIOHardware implements IntakeIO {
  private final TalonFX extendMotor;
  private final TalonFXSignals extendSignals;

  private final SparkFlex intakeMotor;
  private final RelativeEncoder intakeEncoder;

  public IntakeIOHardware(int extendId, int intakeId) {
    this.extendMotor = new TalonFX(extendId);
    this.intakeMotor = new SparkFlex(intakeId, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    var intakeConfig = new SparkFlexConfig();
    intakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(80);
    intakeConfig.encoder.positionConversionFactor(IntakeConstants.intakeGearRatio);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    var extendConfig = new TalonFXConfiguration();
    extendConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.45);
    extendConfig.Feedback.withSensorToMechanismRatio(IntakeConstants.extendGearRatio);
    extendConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    extendConfig.withSlot0(IntakeConstants.extendGains.toSlot0Configs());
    extendConfig.CurrentLimits.withStatorCurrentLimit(15);
    extendConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(IntakeConstants.maxExtension.in(Inches))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(IntakeConstants.maxRetraction.in(Inches));
    extendMotor.getConfigurator().apply(extendConfig);
    extendMotor.setPosition(0.0);

    extendSignals = PhoenixSync.registerTalonFX(extendMotor, 50);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extendConnected = extendSignals.isConnected();
    // Position is in inches due to SensorToMechanismRatio converting rotations to inches
    inputs.extendPosition = Inches.of(extendSignals.getPosition().in(Rotations));
    inputs.extendVelocity = extendSignals.getVelocity();
    inputs.extendVoltageApplied = extendSignals.getVoltage();
    inputs.extendCurrent = extendSignals.getCurrent();

    inputs.intakeConnected = intakeMotor.getLastError() == REVLibError.kOk;
    inputs.intakeVoltageApplied =
        Volts.of(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    inputs.intakeCurrent = Amps.of(intakeMotor.getOutputCurrent());
    inputs.intakeVelocity = RPM.of(intakeEncoder.getVelocity());
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (RobotState.isDisabled()) {
      extendMotor
          .getConfigurator()
          .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    } else {
      extendMotor
          .getConfigurator()
          .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }
    Logger.recordOutput("Intake/extensionSetpoint", outputs.extendVoltage);
    Logger.recordOutput("Intake/intakeSetpoint", outputs.intakeVoltage);

    extendMotor.setVoltage(outputs.extendVoltage.in(Volts));
    intakeMotor.setVoltage(outputs.intakeVoltage.in(Volts));
  }
}
