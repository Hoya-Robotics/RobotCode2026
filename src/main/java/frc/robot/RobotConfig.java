package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.configs.Slot0Configs;

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
			return new Slot0Configs()
				.withKP(kp)
				.withKI(ki)
				.withKD(kd);
		}
  }

  public record CameraConfig(
      String name, Transform3d robotToCamera, SimCameraProperties simProps) {}

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
    public static final Translation2d[] moduleTranslations = new Translation2d[4];

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
    public static final int spinMotorId = 20;
    public static final int rampMotorId = 21;
  }

	public static final class TurretConstants {
		public static final PIDGains hoodGains = new PIDGains(0.0, 0.0, 0.0);
		public static final PIDGains yawGains = new PIDGains(0.0, 0.0, 0.0);
		public static final PIDGains shootGains = new PIDGains(0.0, 0.0, 0.0);
	}

  // Vision Constants
  public static final class VisionConstants {
    public static final boolean enableLimelightRewind = true;
    public static final List<CameraConfig> cameras = new ArrayList<>();
    public static final int matchImuMode = 4;
    public static final Distance multitagTagDistanceThreshold = Meters.of(3.25);
  }

  // Simulated Robot Constants
  public static final class SimConstants {
    public static final double drivetrainSimLoopPeriod = 0.005; // 5ms

    public static final double[] photonSimStdDevs = new double[] {0.015, 0.015, 0.015};
    public static final SimCameraProperties LL4CameraProperties = new SimCameraProperties();
  }

  static {
    double dx = trackWidthX.in(Meters) / 2.0;
    double dy = trackWidthY.in(Meters) / 2.0;

    DriveConstants.moduleTranslations[0] = new Translation2d(dx, dy);
    DriveConstants.moduleTranslations[1] = new Translation2d(dx, -dy);
    DriveConstants.moduleTranslations[2] = new Translation2d(-dx, dy);
    DriveConstants.moduleTranslations[3] = new Translation2d(-dx, -dy);

    SimConstants.LL4CameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
    SimConstants.LL4CameraProperties.setFPS(128);
    SimConstants.LL4CameraProperties.setAvgLatencyMs(5);
    SimConstants.LL4CameraProperties.setLatencyStdDevMs(2);
    SimConstants.LL4CameraProperties.setCalibError(0.25, 0.08);
  }
}
