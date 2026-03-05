package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public class HoodIOInputs {
    public boolean isConnected = false;
    public double voltageApplied = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double positionRads = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setAngle(Angle angle) {}
}
