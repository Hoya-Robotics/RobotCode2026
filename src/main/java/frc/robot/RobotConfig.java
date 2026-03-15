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
import java.util.Optional;
import java.util.function.BooleanSupplier;

/*
 * Complete description of physical and virtual robot configuration
 */
public class RobotConfig {
  public enum TurretTarget {
    HUB,
    ON_THE_MOVE,
    NEAREST_TAG,
    PASSING,
    TUNING,
    CONSTANT_FORWARD,
    FRONT_OF_HUB
  }

  public enum SuperStructureState {
    IDLE,
    INTAKE,
    SHOOT,
    REVERSE_INTAKE
  }

  public enum OperationMode {
    REAL,
    SIM,
  }

  // TODO: make this be tunable in advantage scope using 'LoggedTunableNumber'
  // - make slot0configs / pid controller wrappers that have an reapply command when constants
  // change
  public record PIDGains(double kp, double ki, double kd) {
    public PIDController toController() {
      return new PIDController(kp, ki, kd);
    }

    public Slot0Configs toSlot0Configs() {
      return new Slot0Configs().withKP(kp).withKI(ki).withKD(kd);
    }
  }

  public record CameraConfig(
      String name, Transform3d robotToCamera, Optional<BooleanSupplier> filter) {}

  public static OperationMode getMode() {
    return RobotBase.isReal() ? OperationMode.REAL : OperationMode.SIM;
  }

  public static final Mass robotMass = Kilograms.of(65.0);

  public static final Distance bumperWidthX = Inches.of(16.75);
  public static final Distance bumperWidthY = Inches.of(16.75);
  public static final Distance trackWidthX = Inches.of(21.75);
  public static final Distance trackWidthY = Inches.of(21.75);

  public static final Distance wheelRadius = Inches.of(1.897);
  public static final Distance drivetrainRadius = Inches.of(15.38);

  public static final class DriveConstants {
    public static final double maxDriveSpeedMps =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxDriveAccelMps = 10.0;
    public static final double maxRotationSpeedRadPerSec =
        maxDriveSpeedMps / drivetrainRadius.in(Meters);

    public static final PIDGains toPoseLinearGains = new PIDGains(3.5, 0.0, 0.0);
    public static final PIDGains toPoseOmegaGains = new PIDGains(3.5, 0.0, 0.0);
    public static final double toPoseLinearTolerance = Units.inchesToMeters(2.0);
    public static final double toPoseThetaTolerance = Units.degreesToRadians(3.0);
    public static final double toPoseEndSpeed = 0.25;

    public static final PIDGains trenchYGains = new PIDGains(3.5, 0.0, 0.0);

    public static final PIDGains choreoXGains = new PIDGains(7.5, 0.0, 0.0);
    public static final PIDGains choreoYGains = new PIDGains(7.5, 0.0, 0.0);
    public static final PIDGains choreoOmegaGains = new PIDGains(4.5, 0.0, 0.0);
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

    public static final Distance maxExtension = Inches.of(11.0);
    public static final Distance maxRetraction = Inches.of(7.5);

    public static final Distance agitateOutDist = Inches.of(10.0);
    public static final Distance agitateInDist = Inches.of(7.5);
  }

  public static final class TurretConstants {
    public static final int hoodMotorId = 40;
    public static final int azimuthMotorId = 41;
    public static final int azimuthEncoderId = 42;
    public static final int launcherMotorId = 43;

    public static final double hoodGearRatio = 171.600;
    public static final double azimuthGearRatio = 42.0;
    public static final double launcherGearRatio = 2.25;

    public static final Distance launcherWheelRadius = Inches.of(2.0);

    public static final PIDGains hoodGains = new PIDGains(180.0, 0.0, 0.0);
    public static final PIDGains azimuthGains = new PIDGains(100.0, 0.0, 5.0);
    // public static final PIDGains shootGains = new PIDGains(60.0, 0.0, 0.0);
    public static final PIDGains shootGains = new PIDGains(0.75, 0.0, 0.0);

    public static final Angle maxAzimuthAngle = Rotations.of(0.73);
    public static final Angle minAzimuthAngle = Rotations.of(-0.35);

    public static final Angle azimuthTolerance = Degrees.of(3.5);
    public static final Angle hoodTolerance = Degrees.of(1.0);
    public static final AngularVelocity shotSpeedTolerance = RotationsPerSecond.of(5.0);

    public static final double shooterIdleVoltage = 1.5;

    public static final double cooldownSeconds = 1.0;

    public static final double azimuthLatencyCompensation = 0.050;
    public static final double maxShootingRobotSpeed = 1.0;
    public static final Transform3d robotToTurret =
        new Transform3d(
            Units.inchesToMeters(-6.0),
            Units.inchesToMeters(-6.0),
            Units.inchesToMeters(18.667),
            new Rotation3d(0.0, 0.0, 0.0));
    // new Rotation3d(0.0, 0.0, Units.rotationsToRadians(0.125)));

    public static final Rotation3d cameraRotation =
        new Rotation3d(0.0, Units.degreesToRadians(30), 0.0);
    public static final double azimuthRadiusMeters = Units.inchesToMeters(7.0733);
  }

  public static final class VisionConstants {
    public static final Transform3d turretRobotToCamera =
        TurretConstants.robotToTurret.plus(
            new Transform3d(Translation3d.kZero, TurretConstants.cameraRotation));
    public static final Transform3d hopperRobotToCamera =
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(9.125),
                -Units.inchesToMeters(11.396),
                Units.inchesToMeters(19.828)),
            new Rotation3d(0.0, Units.degreesToRadians(20.0), 0.0));

    public static final List<Integer> hubTags = List.of(9, 10, 25, 26);
    public static final boolean rewindEnabled = true;

    // Hybrid stddev tuning
    public static final double baseStddevMultiplier = 0.9;
    public static final double maxReliableDistance = 4.0;
    public static final double distanceScalingExponent = 2.0;
    public static final double singleTagPenalty = 2.5;

    // Filtering thresholds
    public static final double maxAcceptableDistance = 5.0;
    public static final double maxAcceptableStddev = 3.5;
  }

  public static final class SimConstants {
    public static final double drivetrainSimLoopPeriod = 0.005; // 5ms
    public static final double azimuthMOI = 0.025; // kg*m², turret moment of inertia
  }
}
