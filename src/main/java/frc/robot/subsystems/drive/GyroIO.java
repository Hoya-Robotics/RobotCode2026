package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    public Rotation2d yaw = Rotation2d.kZero;
    public Rotation2d pitch = Rotation2d.kZero;
    public Rotation2d roll = Rotation2d.kZero;

    public double yawRadPs = 0.0;
    public double pitchRadps = 0.0;
    public double rollRadps = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
