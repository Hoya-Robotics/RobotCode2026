package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;

public class LauncherIOHardware implements LauncherIO {
  private final TalonFX motor;
  private final TalonFXSignals signals;

  private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0.0);

  // private VelocityVoltage request = new VelocityVoltage(0.0);

  public LauncherIOHardware(int motorId) {
    this.motor = new TalonFX(motorId);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(new Slot0Configs().withKS(12.3).withKV(0.36).withKP(30));
    // config.withSlot0(TurretConstants.shootGains.toSlot0Configs().withKV(0.84 /
    // 3.0).withKA(0.16));
    config.Feedback.withSensorToMechanismRatio(TurretConstants.launcherGearRatio);
    config.CurrentLimits.withStatorCurrentLimit(100);
    config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    motor.getConfigurator().apply(config);

    this.signals = PhoenixSync.registerTalonFX(motor, 150);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.isConnected = signals.isConnected();
    inputs.voltageApplied = signals.getVoltage();
    inputs.current = signals.getCurrent();
    inputs.velocity = signals.getVelocity();
    inputs.position = signals.getPosition();
  }

  @Override
  public void setSpeed(AngularVelocity velocity) {
    motor.setControl(request.withVelocity(velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
