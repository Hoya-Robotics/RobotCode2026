package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;

public class TurretIOHardware implements TurretIO {
  private final TalonFX hoodMotor;
  private final TalonFX yawMotor;
  private final TalonFX shootMotor;

  private final CANcoder yawEncoder;

  private PositionTorqueCurrentFOC hoodRequest = new PositionTorqueCurrentFOC(0.0);
  private PositionTorqueCurrentFOC yawRequest = new PositionTorqueCurrentFOC(0.0);
	private VelocityTorqueCurrentFOC shootRequest = new VelocityTorqueCurrentFOC(0.0);

	private final TalonFXSignals hoodSignals;
	private final TalonFXSignals yawSignals;
	private final TalonFXSignals shootSignals;

  public TurretIOHardware(int hoodId, int yawId, int yawEncoderId, int shootId) {
    hoodMotor = new TalonFX(hoodId);
    yawMotor = new TalonFX(yawId);
    shootMotor = new TalonFX(shootId);
    yawEncoder = new CANcoder(yawEncoderId);

    var shootConfig = new TalonFXConfiguration();
		shootConfig.withSlot0(TurretConstants.shootGains.toSlot0Configs());
    shootConfig.CurrentLimits.withStatorCurrentLimit(0.0).withSupplyCurrentLimit(0.0);
    shootConfig
        .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    var hoodConfig = new TalonFXConfiguration();
		hoodConfig.withSlot0(TurretConstants.hoodGains.toSlot0Configs());
    hoodConfig
        .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    var yawEncoderConfig = new CANcoderConfiguration();
    yawEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    yawEncoderConfig.MagnetSensor.withMagnetOffset(0.0);

    var yawConfig = new TalonFXConfiguration();
		yawConfig.withSlot0(TurretConstants.yawGains.toSlot0Configs());
    yawConfig
        .MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    yawConfig.Feedback.FeedbackRemoteSensorID = yawEncoderId;
    yawConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    yawConfig.Feedback.SensorToMechanismRatio = 0.0;
    yawConfig.Feedback.RotorToSensorRatio = 0.0;

    shootMotor.getConfigurator().apply(shootConfig);
    hoodMotor.getConfigurator().apply(hoodConfig);
    yawEncoder.getConfigurator().apply(yawEncoderConfig);
    yawMotor.getConfigurator().apply(yawConfig);

		hoodSignals = PhoenixSync.registerTalonFX(hoodMotor, 100);
		yawSignals = PhoenixSync.registerTalonFX(yawMotor, 100);
		shootSignals = PhoenixSync.registerTalonFX(shootMotor, 100);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
		inputs.hoodConnected = hoodSignals.isConnected();
		inputs.hoodVoltageApplied = hoodSignals.getVoltage();
		inputs.hoodVelocityRadsPerSec = hoodSignals.getVelocityRadsPerSec();
		inputs.hoodPositionRad = hoodSignals.getPositionRads();

		inputs.yawConnected = yawSignals.isConnected();
		inputs.yawVoltageApplied = yawSignals.getVoltage();
		inputs.yawVelocityRadsPerSec = yawSignals.getVelocityRadsPerSec();
		inputs.yawPositionRad = yawSignals.getPositionRads();

		inputs.shootConnected = shootSignals.isConnected();
		inputs.shootVoltageApplied = shootSignals.getVoltage();
		inputs.shootVelocityRadsPerSec = shootSignals.getVelocityRadsPerSec();
		inputs.shootPositionRad = shootSignals.getPositionRads();
	}

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    hoodMotor.setControl(hoodRequest.withPosition(outputs.hoodAngle));
    yawMotor.setControl(yawRequest.withPosition(outputs.yawAngle));
		shootMotor.setControl(shootRequest.withVelocity(outputs.flywheelSpeed));
  }
}
