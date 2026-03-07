package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public class LauncherIOInputs {
    public boolean isConnected = false;
    public Voltage voltageApplied = Volts.zero();
    public Current current = Amps.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Angle position = Radians.zero();
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void setSpeed(AngularVelocity velocity) {}
}
