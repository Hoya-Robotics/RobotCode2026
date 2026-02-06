package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.photonvision.simulation.SimCameraProperties;

/*
 * Complete description of physical and virtual robot configuration
 */
public class RobotConfig {
  public enum OperationMode {
    REAL,
    SIM,
  }

  public record PIDGains(double kp, double ki, double kd) {
    public PIDController toController() {
      return new PIDController(kp, ki, kd);
    }
  }

  public enum CameraType {
    POSE_ESTIMATE,
    FUEL_DETECT,
    HUB_ESTIMATE
  }

  public record CameraConfig(String name, Transform3d robotToCamera, CameraType type) {}

  public static OperationMode getMode() {
    return RobotBase.isReal() ? OperationMode.REAL : OperationMode.SIM;
  }

  // Robot dimensions / specs
  public static final Mass robotMass = Kilograms.of(65.0);

  public static final Distance bumperWidthX = Inches.of(30.0);
  public static final Distance bumperWidthY = Inches.of(30.0);
  public static final Distance trackWidthX = Inches.of(24.4);
  public static final Distance trackWidthY = Inches.of(24.4);

  public static final Distance wheelRadius = Inches.of(1.75);

  // Drivebase Constants/Config
  public static final class DriveConstants {
    public static final Translation2d[] moduleTranslations = new Translation2d[4];
    public static final SwerveModuleState[] xBrakeStates = new SwerveModuleState[4];

    public static final double maxDriveSpeedMps = 4.73;
    public static final double maxDriveAccelMps = 9.0;
    public static final double maxRotationSpeedRps = 10.0;
    public static final double maxRotationAccelRps = 15.0;

    public static final PIDGains toPoseLinearGains = new PIDGains(0.35, 0.0, 0.0);
    public static final double toPoseLinearTolerance = Units.inchesToMeters(2.0);
    public static final PIDGains toPoseOmegaGains = new PIDGains(0.8, 0.0, 0.0);
    public static final double toPoseThetaTolerance = Units.degreesToRadians(3.0);

    public static final PIDGains choreoXGains = new PIDGains(10.0, 0.0, 0.0);
    public static final PIDGains choreoYGains = new PIDGains(10.0, 0.0, 0.0);
    public static final PIDGains choreoThetaGains = new PIDGains(7.0, 0.0, 0.0);

    public static final double controllerDeadband = 0.1;

    public static final double driveKs = 0.05;
    public static final double driveKv = 4.0;

    public static final double trajectoryLookeadDist = 0.35;
  }

  // Vision Constants
  public static final class VisionConstants {
    public static final boolean enableLimelightRewind = true;
    public static final List<CameraConfig> cameras = new ArrayList<>();
    public static final int matchImuMode = 4;
    public static final Distance multitagTagDistanceThreshold = Meters.of(3.25);
  }

  // Shooter Constants

  public static final class ShooterConstants {
    public static final Vector<N3> trajectoryWeights =
        VecBuilder.fill(0.8, 10.0, 0.1); // Weights: { shot speed, pitch error, time of flight }
    public static final Angle optimalPitch = Degrees.of(50.0);
    public static final double lookaheadSeconds = 0.1;
    public static final Transform3d robotToTurret =
        new Transform3d(0.0, 0.0, Units.inchesToMeters(25), Rotation3d.kZero);
    public static final Distance hubFunnelClearance = Meters.of(2.1 - robotToTurret.getZ());
  }

  // Simulated Robot Constants
  public static final class SimConstants {
    public static final DriveTrainSimulationConfig mapleSwerveConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMass)
            .withCustomModuleTranslations(DriveConstants.moduleTranslations)
            .withTrackLengthTrackWidth(trackWidthX, trackWidthY)
            .withGyro(COTS.ofPigeon2());

    public static final double[] photonSimStdDevs = new double[] {0.015, 0.015, 0.015};

    public static final PIDGains simDriveMotorGains = new PIDGains(0.1, 0.0, 0.0);
    public static final PIDGains simSteerMotorGains = new PIDGains(10.0, 0.0, 0.0);

    public static final SimCameraProperties LL4CameraProperties = new SimCameraProperties();
  }

  static {
    double dx = trackWidthX.in(Meters) / 2.0;
    double dy = trackWidthY.in(Meters) / 2.0;

    DriveConstants.moduleTranslations[0] = new Translation2d(dx, dy);
    DriveConstants.moduleTranslations[1] = new Translation2d(dx, -dy);
    DriveConstants.moduleTranslations[2] = new Translation2d(-dx, dy);
    DriveConstants.moduleTranslations[3] = new Translation2d(-dx, -dy);

    DriveConstants.xBrakeStates[0] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    DriveConstants.xBrakeStates[1] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));
    DriveConstants.xBrakeStates[2] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    DriveConstants.xBrakeStates[3] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));

    SimConstants.LL4CameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
    SimConstants.LL4CameraProperties.setFPS(128);
    SimConstants.LL4CameraProperties.setAvgLatencyMs(5);
    SimConstants.LL4CameraProperties.setLatencyStdDevMs(2);
    SimConstants.LL4CameraProperties.setCalibError(0.25, 0.08);
  }
}
