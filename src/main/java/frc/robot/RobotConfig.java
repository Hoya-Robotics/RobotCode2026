package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.TunerConstants;
import java.util.List;

/*
 * Complete description of physical and virtual robot configuration
 */
public class RobotConfig {
  public enum TurretTarget {
    HUB,
    NEAREST_TAG,
    PASSING,
    TUNING
  }

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

  public static final Mass robotMass = Kilograms.of(65.0);

  public static final Distance bumperWidthX = Inches.of(30.0);
  public static final Distance bumperWidthY = Inches.of(30.0);
  public static final Distance trackWidthX = Inches.of(21.75);
  public static final Distance trackWidthY = Inches.of(21.75);

  public static final Distance wheelRadius = Inches.of(1.897);
  public static final Distance drivetrainRadius = Inches.of(15.38);

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

    public static final PIDGains trenchYGains = new PIDGains(5.0, 0.0, 0.0);

		public static final PIDGains choreoXGains = new PIDGains(0.0, 0.0, 0.0);
		public static final PIDGains choreoYGains = new PIDGains(0.0, 0.0, 0.0);
		public static final PIDGains choreoOmegaGains = new PIDGains(0.0, 0.0, 0.0);
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

    public static final Distance maxExtension = Inches.of(11.4);
    public static final Distance maxRetraction = Inches.of(7.5);

    public static final Distance agitateOutDist = Inches.of(10.0);
    public static final Distance agitateInDist = Inches.of(9.0);
  }

  public static final class TurretConstants {
    public static final int hoodMotorId = 40;
    public static final int azimuthMotorId = 41;
    public static final int azimuthEncoderId = 42;
    public static final int launcherMotorId = 43;

    public static final double hoodGearRatio = 171.600;
    public static final double azimuthGearRatio = 42.0;
    public static final double launcherGearRatio = 3.0;

    public static final PIDGains hoodGains = new PIDGains(180.0, 0.0, 0.0);
    public static final PIDGains azimuthGains = new PIDGains(100.0, 0.0, 5.0);
    public static final PIDGains shootGains = new PIDGains(0.0, 0.0, 0.0);

    public static final Angle azimuthTolerance = Degrees.of(1.5);
    public static final Angle hoodTolerance = Degrees.of(0.5);
    public static final AngularVelocity shotSpeedThreshold = RotationsPerSecond.of(20);
    public static final double shooterWarmVoltage = 3.0;

    public static final double azimuthLatencyCompensation = 0.050;
    public static final Transform3d robotToTurret =
        new Transform3d(
            Units.inchesToMeters(-6.0),
            Units.inchesToMeters(-6.0),
            Units.inchesToMeters(18.667),
            new Rotation3d(0.0, 0.0, Units.rotationsToRadians(0.125)));

    public static final Rotation3d cameraRotation =
        new Rotation3d(0.0, Units.degreesToRadians(30), 0.0);
    public static final double azimuthRadiusMeters = Units.inchesToMeters(7.0733);
  }

  public static final class VisionConstants {
    public static final CameraConfig turretCameraConfig =
        new CameraConfig(
            "limelight-turret",
            TurretConstants.robotToTurret.plus(
                new Transform3d(Translation3d.kZero, TurretConstants.cameraRotation)));

    public static final List<Integer> hubTags = List.of(9, 10, 25, 26);
    public static final boolean rewindEnabled = false;

    // Hybrid stddev tuning
    public static final double baseStddevMultiplier = 1.0;
    public static final double maxReliableDistance = 5.0; // meters
    public static final double distanceScalingExponent = 2.0;
    public static final double singleTagPenalty = 2.5;

    // Filtering thresholds
    public static final double maxAcceptableDistance = 6.0; // meters
    public static final double maxAcceptableStddev = 2.0; // meters

    // Hub-relative blending
    public static final boolean useHubLocalizationBlending = false;
    public static final double hubRelativeMaxDistance = 3.5; // prefer hub-relative under this
    public static final double hubObservationTimeout = 0.5; // seconds
  }

  public static final class SimConstants {
    public static final double drivetrainSimLoopPeriod = 0.005; // 5ms
  }
}
