package frc.robot.subsystems.launcher;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  public Launcher(LauncherIO io) {
    this.io = io;
  }

  public AngularVelocity getSpeed() {
    return inputs.velocity;
  }

  public void setSpeed(AngularVelocity velocity) {
    Logger.recordOutput("Launcher/setpoint", velocity);
    this.io.setSpeed(velocity);
  }

  public void setVoltage(double voltage) {
    Logger.recordOutput("Launcher/setpoint", voltage);
    this.io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
    Logger.recordOutput("Launcher/velocityRPS", getSpeed().in(Units.RotationsPerSecond));
  }
}
