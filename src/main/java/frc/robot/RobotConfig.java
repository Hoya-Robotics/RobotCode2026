package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

/*
 * Complete description of physical and virtual robot configuration
 */
public class RobotConfig {
  public enum OperationMode {
    REAL,
    SIM,
  }

  public record PIDConfig(double kp, double ki, double kd, double tolerance) {}

  public static OperationMode getMode() {
    return RobotBase.isReal() ? OperationMode.REAL : OperationMode.SIM;
  }

  // Robot dimensions / specs
  public static Mass robotMass = Kilograms.of(0.0);
  public static Distance bumperWidthX = Inches.of(0.0);
  public static Distance bumperWidthY = Inches.of(0.0);
  public static Distance trackWidthX = Inches.of(24.4);
  public static Distance trackWidthY = Inches.of(24.4);

  // Drivebase Constants/Config
  public static Translation2d[] moduleTranslations = new Translation2d[4];
  public static Distance wheelRadius = Inches.of(1.75);

  public static double maxDriveSpeedMps = 7.5;
  public static double maxRotationSpeedRps = 10.0;

  public static PIDConfig toPoseLinearGains = new PIDConfig(0.0, 0.0, 0.0, 0.0);
  public static PIDConfig toPoseOmegaGains = new PIDConfig(0.0, 0.0, 0.0, 0.0);

  public static double controllerDeadband = 0.1;

  public static double driveKs = 0.0;
  public static double driveKv = 0.0;

  static {
    double dx = trackWidthX.in(Meters) / 2.0;
    double dy = trackWidthY.in(Meters) / 2.0;

    moduleTranslations[0] = new Translation2d(dx, dy);
    moduleTranslations[0] = new Translation2d(dx, -dy);
    moduleTranslations[0] = new Translation2d(-dx, dy);
    moduleTranslations[0] = new Translation2d(-dx, -dy);
  }
}
