package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public class LauncherIOInputs {
    public boolean isConnected = false;
    public double voltageApplied = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double positionRads = 0.0;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void setSpeed(AngularVelocity velocity) {}
}
