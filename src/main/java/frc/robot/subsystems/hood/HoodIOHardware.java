package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.PhoenixSync;
import frc.robot.util.PhoenixSync.TalonFXSignals;

public class HoodIOHardware implements HoodIO {
  private final TalonFX motor;
  private final TalonFXSignals signals;

  private PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0.0);

  public HoodIOHardware(int motorId) {
    this.motor = new TalonFX(motorId);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.withSlot0(TurretConstants.hoodGains.toSlot0Configs());
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    motor.getConfigurator().apply(config);

    this.signals = PhoenixSync.registerTalonFX(motor, TurretConstants.hoodGearRatio, 50);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.isConnected = signals.isConnected();
    inputs.voltageApplied = signals.getVoltage();
    inputs.velocityRadsPerSec = signals.getVelocityRadsPerSec();
    inputs.positionRads = signals.getPositionRads();
  }

  public void setAngle(Angle angle) {
    motor.setControl(request.withPosition(angle));
  }
}
