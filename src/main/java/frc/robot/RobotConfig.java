package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.TunerConstants;

/*
 * Complete description of physical and virtual robot configuration
 */
public class RobotConfig {
  public enum OperationMode {
    REAL,
    SIM,
  }

  // TODO: make this be tunable in advantage scope using 'LoggedTunableNumber'
  public record PIDGains(double kp, double ki, double kd) {
    public PIDController toController() {
      return new PIDController(kp, ki, kd);
    }

    public Slot0Configs toSlot0Configs() {
      return new Slot0Configs().withKP(kp).withKI(ki).withKD(kd);
    }
  }

  public record CameraConfig(String name, Transform3d robotToCamera) {}

  public static OperationMode getMode() {
    return RobotBase.isReal() ? OperationMode.REAL : OperationMode.SIM;
  }

  // Robot dimensions / specs
  public static final Mass robotMass = Kilograms.of(65.0);

  // TODO: inertia, https://choreo.autos/usage/estimating-moi/

  public static final Distance bumperWidthX = Inches.of(30.0);
  public static final Distance bumperWidthY = Inches.of(30.0);
  public static final Distance trackWidthX = Inches.of(21.75);
  public static final Distance trackWidthY = Inches.of(21.75);

  public static final Distance wheelRadius = Inches.of(1.897);
  public static final Distance drivetrainRadius = Inches.of(15.38);

  // Drivebase Constants/Config
  public static final class DriveConstants {
    public static final double maxDriveSpeedMps =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxDriveAccelMps = 9.0;
    public static final double maxRotationSpeedRadPerSec =
        maxDriveSpeedMps / drivetrainRadius.in(Meters);

    public static final PIDGains toPoseLinearGains = new PIDGains(5.0, 0.0, 0.0);
    public static final double toPoseLinearTolerance = Units.inchesToMeters(2.0);
    public static final PIDGains toPoseOmegaGains = new PIDGains(8.0, 0.0, 0.0);
    public static final double toPoseThetaTolerance = Units.degreesToRadians(3.0);

    public static final PIDGains trenchYGains = new PIDGains(0.0, 0.0, 0.0);

    public static final PIDGains choreoLinearGains = new PIDGains(7.0, 0.0, 0.0);
    public static final PIDGains choreoThetaGains = new PIDGains(5.0, 0.0, 0.0);
  }

  public static final class SpindexerConstants {
    public static final int indexMotorId = 20;
    public static final int feedMotorId = 21;

    public static final double feedGearRatio = 1.0;
    public static final double indexGearRatio = 3.0;
  }

  public static final class IntakeConstants {
    public static final int extendMotorId = 30;
    public static final int intakeMotorId = 31;

    public static final double extensionRadius = Units.inchesToMeters(0.7);
    public static final double intakeGearRatio = 1.8667;
    public static final double extendGearRatio = 1.8364;
    public static final PIDGains extendGains = new PIDGains(1.05, 0.0, 0.0);

    public static final double maxExtensionMeters = Units.inchesToMeters(10.9 + 0.25);
    public static final double maxRetractionMeters = Units.inchesToMeters(6.5);
  }

  public static final class TurretConstants {
    public static final int hoodMotorId = 40;
    public static final int azimuthMotorId = 41;
    public static final int azimuthEncoderId = 42;
    public static final int launcherMotorId = 43;

    public static final double hoodGearRatio = 171.600;
    public static final double azimuthGearRatio = 42.0;
    public static final double launcherGearRatio = 3.0;

    // TODO: gains + transform
    public static final PIDGains hoodGains = new PIDGains(180.0, 0.0, 0.0);
    public static final PIDGains azimuthGains = new PIDGains(100.0, 0.0, 5.0);
    public static final PIDGains shootGains = new PIDGains(0.0, 0.0, 0.0);
    public static final Transform3d robotToTurret =
        new Transform3d(
            Units.inchesToMeters(6.0),
            Units.inchesToMeters(6.0),
            Units.inchesToMeters(12.0),
            Rotation3d.kZero);
  }

  public static final class SimConstants {
    public static final double drivetrainSimLoopPeriod = 0.005; // 5ms
  }
}
