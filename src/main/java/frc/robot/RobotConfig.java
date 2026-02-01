package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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

/*
 * Complete description of physical and virtual robot configuration
 */
public class RobotConfig {
  public enum OperationMode {
    REAL,
    SIM,
  }

  public record PIDGains(double kp, double ki, double kd) {}

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
  public static final Translation2d[] moduleTranslations = new Translation2d[4];
  public static final SwerveModuleState[] xBrakeStates = new SwerveModuleState[4];

  public static final double maxDriveSpeedMps = 7.5;
  public static final double maxRotationSpeedRps = 10.0;

  public static final PIDGains toPoseLinearGains = new PIDGains(0.75, 0.0, 0.0);
  public static final double toPoseLinearTolerance = Units.inchesToMeters(2.0);
  public static final PIDGains toPoseOmegaGains = new PIDGains(0.8, 0.0, 0.0);
  public static final double toPoseThetaTolerance = Units.degreesToRadians(3.0);

  public static final double controllerDeadband = 0.1;

  public static final double driveKs = 0.0;
  public static final double driveKv = 2.0;

  // Vision Constants

  public static final List<CameraConfig> cameras = new ArrayList<>();
  public static final Transform2d[] cameraToRobot2d;

  // Shooter Constants

  // Weights: { shot speed, pitch error, time of flight }
  public static final Vector<N3> trajectoryWeights = VecBuilder.fill(0.8, 10.0, 0.1);
  public static final Angle optimalPitch = Degrees.of(50.0);
  public static final double lookaheadSeconds = 0.1;
  public static final Transform3d robotToTurret =
      new Transform3d(0.0, 0.0, Units.inchesToMeters(25), Rotation3d.kZero);
  public static final Distance hubFunnelClearance = Meters.of(2.1 - robotToTurret.getZ());

  // Simulated Robot Constants
  public static final DriveTrainSimulationConfig mapleSwerveConfig;

  public static final PIDGains simDriveMotorGains = new PIDGains(0.1, 0.0, 0.0);
  public static final PIDGains simSteerMotorGains = new PIDGains(10.0, 0.0, 0.0);

  static {
    double dx = trackWidthX.in(Meters) / 2.0;
    double dy = trackWidthY.in(Meters) / 2.0;

    moduleTranslations[0] = new Translation2d(dx, dy);
    moduleTranslations[1] = new Translation2d(dx, -dy);
    moduleTranslations[2] = new Translation2d(-dx, dy);
    moduleTranslations[3] = new Translation2d(-dx, -dy);

    xBrakeStates[0] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    xBrakeStates[1] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));
    xBrakeStates[2] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    xBrakeStates[3] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));

    mapleSwerveConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMass)
            .withCustomModuleTranslations(moduleTranslations)
            .withTrackLengthTrackWidth(trackWidthX, trackWidthY)
            .withGyro(COTS.ofPigeon2());

    cameras.add(
        new CameraConfig(
            "intake",
            new Transform3d(
                Units.inchesToMeters(9.75),
                Units.inchesToMeters(-5.806),
                Units.inchesToMeters(-10.5),
                Rotation3d.kZero),
            CameraType.FUEL_DETECT));

    cameraToRobot2d = new Transform2d[cameras.size()];
    for (int i = 0; i < cameras.size(); ++i) {
      var inv = cameras.get(i).robotToCamera().inverse();
      cameraToRobot2d[i] =
          new Transform2d(inv.getX(), inv.getY(), inv.getRotation().toRotation2d());
    }
  }
}
