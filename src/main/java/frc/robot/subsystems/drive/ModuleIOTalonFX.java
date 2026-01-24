package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public abstract class ModuleIOTalonFX implements ModuleIO {
  protected final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;
  protected final TalonFX driveTalon;
  protected final TalonFX turnTalon;
  protected final CANcoder cancoder;

  protected final StatusSignal<Angle> drivePosition;
  protected final StatusSignal<AngularVelocity> driveVelocity;

  protected final StatusSignal<Angle> turnPositionAbsolute;
  protected final StatusSignal<AngularVelocity> turnVelocity;

  // Drive control signals (velocity control)
  protected PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  protected VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Steer control signals (position control)
  protected PositionTorqueCurrentFOC positionTorqueRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  protected VelocityTorqueCurrentFOC velocityTorqueRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
  protected MotionMagicVoltage positionMagicVoltageRequest = new MotionMagicVoltage(0.0);
  protected MotionMagicTorqueCurrentFOC positionMagicTorqueRequest =
      new MotionMagicTorqueCurrentFOC(0.0);

  private final Debouncer driveStatusDebouncer = new Debouncer(0.5);
  private final Debouncer turnStatusDebouncer = new Debouncer(0.5);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    driveTalon = new TalonFX(constants.DriveMotorId);
    turnTalon = new TalonFX(constants.SteerMotorId);
    cancoder = new CANcoder(constants.EncoderId);

    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;

    // Limit current to prevent slippage
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveTalon.getConfigurator().apply(driveConfig);
    driveTalon.setPosition(0.0);

    var turnConfig = constants.SteerMotorInitialConfigs;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Fuse cancoder sensor data
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> FeedbackSensorSourceValue.RemoteCANcoder;
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turnTalon.getConfigurator().apply(turnConfig);

    // Motion magic profile gains
    var motionMagicConfigs = turnConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
    motionMagicConfigs.MotionMagicAcceleration = 0.0;
    motionMagicConfigs.MotionMagicJerk = 0.0;
    motionMagicConfigs.MotionMagicExpo_kV = 0.0;
    motionMagicConfigs.MotionMagicExpo_kA = 0.0;

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html
    CANcoderConfiguration encoderConfig = constants.EncoderInitialConfigs;
    encoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    encoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(encoderConfig);

    driveVelocity = driveTalon.getVelocity();
    drivePosition = driveTalon.getPosition();
    turnPositionAbsolute = cancoder.getAbsolutePosition();
    turnVelocity = turnTalon.getVelocity();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    var driveStatus = BaseStatusSignal.refreshAll(driveVelocity, drivePosition);
    var turnStatus = BaseStatusSignal.refreshAll(turnVelocity, turnPositionAbsolute);

    inputs.driveConnected = driveStatusDebouncer.calculate(driveStatus.isOK());
    inputs.driveVelocity =
        RadiansPerSecond.of(
            driveVelocity.getValue().in(RadiansPerSecond) / constants.DriveMotorGearRatio);
    inputs.drivePosition =
        Meters.of(drivePosition.getValue().in(Radians) / constants.DriveMotorGearRatio);

    inputs.turnConnected = turnStatusDebouncer.calculate(turnStatus.isOK());
    inputs.absoluteTurnHeading = new Rotation2d(turnPositionAbsolute.getValue());
    inputs.turnVelocity = turnVelocity.getValue();
  }

  @Override
  public void applyOutputs(ModuleIOOutputs outputs) {
    final double driveVelocity =
        outputs.driveVelocity.in(RotationsPerSecond) * constants.DriveMotorGearRatio;

    // Velocity control for drive
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage ->
              velocityVoltageRequest
                  .withVelocity(driveVelocity)
                  .withFeedForward(outputs.feedforward);
          case TorqueCurrentFOC ->
              velocityTorqueRequest
                  .withVelocity(driveVelocity)
                  .withFeedForward(outputs.feedforward);
        });

    // Position control for steer
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage ->
              DriveConstants.motionMagicSteerControl
                  ? positionMagicVoltageRequest.withPosition(outputs.turnHeading.getMeasure())
                  : positionVoltageRequest.withPosition(outputs.turnHeading.getMeasure());
          case TorqueCurrentFOC ->
              DriveConstants.motionMagicSteerControl
                  ? positionMagicTorqueRequest.withPosition(outputs.turnHeading.getMeasure())
                  : positionTorqueRequest.withPosition(outputs.turnHeading.getMeasure());
        });
  }
}
