package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;

    public double drivePositionMeters = 0.0;

    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveVoltsApplied = 0.0;
    public double driveTempCelsius = 0.0;

    public boolean steerConnected = false;

    public Rotation2d absoluteSteerHeading = Rotation2d.kZero;

    public double steerStatorCurrentAmps = 0.0;
    public double steerSupplyCurrentAmps = 0.0;
    public double steerVoltsApplied = 0.0;
    public double steerTempCelsius = 0.0;
  }

  public static class ModuleIOOutputs {
    public double driveVelocityRadps = 0.0;
    public double driveFeedforward = 0.0;
    public Rotation2d steerHeading = Rotation2d.kZero;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void applyOutputs(ModuleIOOutputs outputs) {}
}
