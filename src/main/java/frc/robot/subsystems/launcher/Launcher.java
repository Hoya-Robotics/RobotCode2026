package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  public Launcher(LauncherIO io) {
    this.io = io;
  }

  public void setSpeed(AngularVelocity velocity) {
    this.io.setSpeed(velocity);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
  }
}
