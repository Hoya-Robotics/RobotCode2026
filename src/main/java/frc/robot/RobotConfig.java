package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.config.ClosedLoopConfig;
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
import frc.robot.util.GenericTunableGains.SparkTunableGains;
import frc.robot.util.GenericTunableGains.TalonTunableGains;

public class RobotConfig {
  public enum SuperStructureState {
    IDLE,
    SHOOT_INTAKE,
    INTAKE,
    SHOOT,
    REVERSE_INTAKE
  }

  public enum OperationMode {
    REAL,
    SIM,
  }

  public record PIDGains(double kp, double ki, double kd) {
    public PIDController toController() {
      return new PIDController(kp, ki, kd);
    }

    public Slot0Configs toSlot0Configs() {
      return new Slot0Configs().withKP(kp).withKI(ki).withKD(kd);
    }

    public ClosedLoopConfig toSparkGains() {
      return new ClosedLoopConfig().p(kp).i(ki).d(kd);
    }
  }

  public record CameraConfig(String name, Transform3d robotToCamera) {}

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
    public enum DriveState {
      IDLE,
      TO_POSE,
      TELEOP,
      CHOREO
    }

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

    public static final PIDGains choreoXGains = new PIDGains(7.5, 0.0, 0.0);
    public static final PIDGains choreoYGains = new PIDGains(7.5, 0.0, 0.0);
    public static final PIDGains choreoOmegaGains = new PIDGains(5.25, 0.0, 0.0);

    public static final double SOTMSpeedFactor = 0.25;
    public static final double SOTMOmegaFactor = 0.125;
    public static final double SOTMAccelLimit = 0.2;

    public static final Angle odometryMaxPitch = Degrees.of(6.0);
  }

  public static final class SpindexerConstants {
    public enum SpindexerState {
      HOLD,
      FEED,
      COOLDOWN,
      REVERSE
    }

    public static final int indexMotorId = 20;
    public static final int feedMotorId = 21;

    public static final double feedGearRatio = 1.0;
    public static final double indexGearRatio = 1.0 / 3.0;

    public static final SparkTunableGains indexGains = new SparkTunableGains("Spindexer/indexer");
    public static final SparkTunableGains feederGains = new SparkTunableGains("Spindexer/feeder");

    static {
      indexGains.registerGain("kp", 0.0002);
      indexGains.registerGain("kv", 0.0054);

      feederGains.registerGain("kp", 0.0002);
      feederGains.registerGain("kv", 0.0018);
      feederGains.registerGain("cruiseVelocity", 1200);
      feederGains.registerGain("maxAcceleration", 1200);
    }
  }

  public static final class IntakeConstants {
    public enum IntakeState {
      IDLE,
      RETRACT,
      INTAKE,
      AGITATE,
      REVERSE
    }

    public static final int extendMotorId = 30;
    public static final int intakeMotorId = 31;
    public static final int intakeEncoderId = 32;

    public static final double extensionRampRate = 0.125;
    public static final double extensionAgitatePeriod = 0.3;

    public static final double extensionRadius = Units.inchesToMeters(0.7);
    public static final double intakeGearRatio = 1.0 / 1.8667;
    public static final double extendGearRatio = 1.8364;
    public static final double intakeKv = 1.1 * (1.0 / 576.8) / intakeGearRatio; // 0.032408;

    public static final SparkTunableGains intakeGains = new SparkTunableGains("Intake/spin");
    public static final TalonTunableGains extendGains = new TalonTunableGains("Intake/extend", 0);

    static {
      intakeGains.registerGain("kp", 0.0002);

      extendGains.registerGain("kp", 3.0);
      extendGains.registerGain("kd", 0.05);
    }

    public static final Distance maxExtension = Inches.of(11.0);
    public static final Distance maxRetraction = Inches.of(5.1); // 7.5
  }

  public static final class TurretConstants {
    public enum TurretState {
      IDLE_TRACK,
      NEAR_TRENCH,
      SHOOT
    }

    public static final int hoodMotorId = 40;
    public static final int azimuthMotorId = 41;
    public static final int azimuthEncoderId = 42;
    public static final int launcherMotorId = 43;

    public static final double hoodGearRatio = 171.600;
    public static final double azimuthGearRatio = 42.0;
    public static final double launcherGearRatio = 2.25;

    public static final TalonTunableGains azimuthGains = new TalonTunableGains("Turret/azimuth", 0);
    public static final TalonTunableGains flywheelGains =
        new TalonTunableGains("Turret/flywheel", 0);
    public static final TalonTunableGains hoodGains = new TalonTunableGains("Turret/hood", 0);

    static {
      hoodGains.registerGain("kp", 170);
      hoodGains.registerGain("ki", 0);
      hoodGains.registerGain("kd", 0);
      hoodGains.registerGain("ks", 4.0);
      hoodGains.registerGain("ka", 0);

      azimuthGains.registerGain("kp", 125);
      azimuthGains.registerGain("kd", 8.0);
      azimuthGains.registerGain("kv", 2.5);
      azimuthGains.registerGain("ka", 2.13);
      azimuthGains.registerGain("ks", 14);

      flywheelGains.registerGain("kv", 0.36);
      flywheelGains.registerGain("ks", 12.3);
      flywheelGains.registerGain("kp", 30.0);
    }

    public static final Angle trenchHoodAngle = Degrees.of(12.0);
    public static final Angle hoodTolerance = Degrees.of(1.5);

    public static final Angle maxAzimuthAngle = Rotations.of(0.73);
    public static final Angle minAzimuthAngle = Rotations.of(-0.35);
    public static final Angle azimuthTolerance = Degrees.of(2.0);
    public static final double azimuthLatencyCompensation = 0.050;
    public static final double azimuthRadiusMeters = Units.inchesToMeters(7.0733);

    public static final AngularVelocity shotSpeedTolerance = RotationsPerSecond.of(2.5);
    public static final AngularVelocity shotIdleSpeed = RotationsPerSecond.of(17.5);
    public static final Distance launcherWheelRadius = Inches.of(2.0);

    public static final double cooldownSeconds = 0.5;
    public static final Transform3d robotToTurret =
        new Transform3d(
            Units.inchesToMeters(-6.0),
            Units.inchesToMeters(-6.0),
            Units.inchesToMeters(18.667),
            new Rotation3d(0.0, 0.0, 0.0));
    public static final Rotation3d cameraRotation =
        new Rotation3d(0.0, Units.degreesToRadians(30), 0.0);
  }

  public static final class VisionConstants {
    public static final CameraConfig turretConfig =
        new CameraConfig(
            "limelight-turret",
            TurretConstants.robotToTurret.plus(
                new Transform3d(Translation3d.kZero, TurretConstants.cameraRotation)));

    public static final CameraConfig hopperConfig =
        new CameraConfig(
            "limelight-hopper",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.125),
                    -Units.inchesToMeters(11.396),
                    Units.inchesToMeters(19.828)),
                new Rotation3d(0.0, Units.degreesToRadians(20.0), 0.0)));

    public static final CameraConfig hopperConfigSim =
        new CameraConfig(
            "limelight-hopper",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.125),
                    -Units.inchesToMeters(11.396),
                    Units.inchesToMeters(19.828)),
                new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0)));

    public static final double zThreshold = 0.2;
    public static final double minSingleTagArea = 1.0;
    public static final double maxSingleTagAmbiguity = 0.19;
  }

  public static final class SimConstants {
    public static final double drivetrainSimLoopPeriod = 0.005;
    public static final double azimuthMOI = 0.025;
  }
}
