package frc.robot.subsystems.launcher;

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

  public LauncherIOHardware(int motorId) {
    this.motor = new TalonFX(motorId);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.shootGains.toSlot0Configs());
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    motor.getConfigurator().apply(config);

    this.signals = PhoenixSync.registerTalonFX(motor, TurretConstants.shootGearRatio, 50);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.isConnected = signals.isConnected();
    inputs.voltageApplied = signals.getVoltage();
    inputs.velocityRadsPerSec = signals.getVelocityRadsPerSec();
    inputs.positionRads = signals.getPositionRads();
  }

  @Override
  public void setSpeed(AngularVelocity velocity) {
    motor.setControl(request.withVelocity(velocity));
  }
}
