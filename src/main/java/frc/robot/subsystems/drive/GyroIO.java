package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    // Do we need pitch or roll?
    public boolean isConnected = false;
    public Rotation2d yaw = Rotation2d.kZero;
    public AngularVelocity yawVelocity = RadiansPerSecond.of(0);
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void setYaw(Angle newYaw) {}
}
