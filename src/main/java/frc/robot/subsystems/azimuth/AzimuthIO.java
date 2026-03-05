package frc.robot.subsystems.azimuth;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface AzimuthIO {
  @AutoLog
  public class AzimuthIOInputs {
    public boolean isConnected = false;
    public double voltageApplied = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double positionRads = 0.0;
  }

  public default void updateInputs(AzimuthIOInputs inputs) {}

  public default void setAngle(Angle angle) {}
}
