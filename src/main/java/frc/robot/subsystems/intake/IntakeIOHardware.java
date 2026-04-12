package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;
import org.littletonrobotics.junction.Logger;

public class IntakeIOHardware implements IntakeIO {
  private final TalonFX extendMotor;
  private final CANcoder extendEncoder;
  private final TalonFXSignals extendSignals;
  private PositionVoltage extendRequest = new PositionVoltage(0.0);
  private VoltageOut extendRawRequest = new VoltageOut(0.0);

  private final SparkFlex intakeMotor;
  private final SparkClosedLoopController intakeController;
  private final RelativeEncoder intakeEncoder;
  private NeutralModeValue lastNeutralMode = null;

  public IntakeIOHardware(int extendId, int extendEncoderId, int intakeId) {
    this.extendMotor = new TalonFX(extendId);
    this.intakeMotor = new SparkFlex(intakeId, MotorType.kBrushless);
    this.extendEncoder = new CANcoder(extendEncoderId);
    intakeController = intakeMotor.getClosedLoopController();
    intakeEncoder = intakeMotor.getEncoder();

    var intakeConfig = new SparkFlexConfig();
    intakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    intakeConfig.encoder.positionConversionFactor(IntakeConstants.intakeGearRatio);
    intakeConfig.encoder.velocityConversionFactor(IntakeConstants.intakeGearRatio);
    intakeConfig.closedLoopRampRate(0.075);
    intakeConfig.closedLoop.feedForward.kV(IntakeConstants.intakeKv);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    var extendEncoderConfig = new CANcoderConfiguration();
    extendEncoderConfig.MagnetSensor.withSensorDirection(
        SensorDirectionValue.CounterClockwise_Positive);
    extendEncoder.getConfigurator().apply(extendEncoderConfig);

    var extendConfig = new TalonFXConfiguration();
    extendConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.075);
    extendConfig.Feedback.withRemoteCANcoder(extendEncoder);
    extendConfig.Feedback.withRotorToSensorRatio(10.3846);
    extendConfig.Feedback.withSensorToMechanismRatio(0.1768);
    extendConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    extendConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    extendConfig.CurrentLimits.withStatorCurrentLimit(24);
    extendConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(IntakeConstants.maxExtension.in(Inches))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(3.5);
    // .withReverseSoftLimitThreshold(IntakeConstants.maxRetraction.in(Inches));
    extendMotor.getConfigurator().apply(extendConfig);
    extendMotor.setPosition(0.0);
    extendEncoder.setPosition(0.0);

    IntakeConstants.intakeGains.registerMotor(intakeMotor);
    IntakeConstants.extendGains.registerMotor(extendMotor);

    extendSignals = PhoenixSync.registerTalonFX(extendMotor, 50);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.extendConnected = extendSignals.isConnected();
    // Position is in inches due to SensorToMechanismRatio converting rotations to inches
    inputs.extendPosition = Inches.of(extendSignals.getNativePosition());
    inputs.extendVelocity = MetersPerSecond.of(extendSignals.getNativeVelocity());
    inputs.extendVoltageApplied = Volts.of(extendSignals.getVoltage());
    inputs.extendCurrent = Amps.of(extendSignals.getCurrent());

    inputs.intakeConnected = intakeMotor.getLastError() == REVLibError.kOk;
    inputs.intakeVoltageApplied =
        Volts.of(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    inputs.intakeCurrent = Amps.of(intakeMotor.getOutputCurrent());
    inputs.intakeVelocity = RPM.of(intakeEncoder.getVelocity());
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    NeutralModeValue desiredMode =
        RobotState.isDisabled() ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    if (desiredMode != lastNeutralMode) {
      extendMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(desiredMode));
      lastNeutralMode = desiredMode;
    }
    Logger.recordOutput("Intake/extensionSetpoint", outputs.extendSetpointInches);
    Logger.recordOutput("Intake/intakeSetpoint", outputs.intakeVelocityRPM);

    extendMotor.setControl(
        switch (outputs.extendControlType) {
          case POSITION -> extendRequest.withPosition(outputs.extendSetpointInches);
          case VOLTAGE -> extendRawRequest.withOutput(outputs.extendVoltage);
        });
    intakeController.setSetpoint(outputs.intakeVelocityRPM, ControlType.kVelocity);
  }
}
