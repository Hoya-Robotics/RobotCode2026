package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public Distance drivePosition = Meters.of(0.0);
    public AngularVelocity driveVelocity = RadiansPerSecond.of(0.0);
    public double driveVoltsApplied = 0.0;

    public boolean turnConnected = false;
    public Rotation2d absoluteTurnHeading = Rotation2d.kZero;
    public AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);
    public double turnVoltsApplied = 0.0;
  }

  public static enum ModuleIOOutputMode {
    BRAKE,
    COAST,
    DRIVE
  }

  public static class ModuleIOOutputs {
    public AngularVelocity driveVelocity = RadiansPerSecond.of(0.0);
    public Rotation2d turnHeading = Rotation2d.kZero;
    public double feedforward = 0.0;
    public ModuleIOOutputMode mode = ModuleIOOutputMode.COAST;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void applyOutputs(ModuleIOOutputs outputs) {}
}
