package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.*;
import frc.robot.util.MiscUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance;

  private Pose2d estimatedPose = Pose2d.kZero;
  private Pose2d odometryPose = Pose2d.kZero;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(RobotConfig.moduleTranslations);
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private ChassisSpeeds odometrySpeeds = new ChassisSpeeds();

  // Singleton pattern
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {
    AutoLogOutputManager.addObject(this);
  }

  public void hardSetOdometry(Pose2d pose) {
    this.odometryPose = pose;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(odometrySpeeds, odometryPose.getRotation());
  }

  @AutoLogOutput(key = "RobotState/odometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  public void addOdometryObservation(OdometryObservation observation) {
    var twist = kinematics.toTwist2d(lastModulePositions, observation.modulePositions());
    lastModulePositions = observation.modulePositions();

    odometrySpeeds = observation.speeds();
    odometryPose = new Pose2d(odometryPose.exp(twist).getTranslation(), observation.gyroYaw());
  }

  public void addVisionObservation(AprilTagObservation observation) {}

  // heuristic placeholder, probably crap
  private static double distanceToTrajectorySpeed(double distanceMeters) {
    final double vMin = 6.0;
    final double vMax = 8.0;
    final double dStart = 1.5;
    final double dEnd = 5.0;
    if (distanceMeters < dStart) {
      return vMin;
    } else if (distanceMeters > dEnd) {
      return vMax;
    } else {
      double t = (distanceMeters - dStart) / (dEnd - dStart);
      return MathUtil.interpolate(vMin, vMax, t);
    }
  }

  // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
  private static final double G = 9.8;

  private static Rotation2d getShotPitch(double v, double x, double y, double k, double h) {
    double mj = Double.MAX_VALUE;
    double best = 0.0;
    for (double theta = 5; theta <= 85; ++theta) {
      double th = Units.degreesToRadians(theta);
      double y_hit = x * Math.tan(th) - ((G * x * x) / (2 * v * v * Math.pow(Math.cos(th), 2)));
      double h_apex = (v * v * Math.pow(Math.sin(th), 2)) / (2 * G);
      double J = Math.pow(y_hit - y, 2) + k * Math.pow(h_apex - h, 2);
      if (J < mj) {
        mj = J;
        best = th;
      }
    }
    return Rotation2d.fromRadians(best);
  }

  public OptimalShot getOptimalShot() {
    var robot = odometryPose;
    var speeds = odometrySpeeds;
    var future =
        robot.exp(
            new Twist2d(
                speeds.vxMetersPerSecond * RobotConfig.lookaheadSeconds,
                speeds.vyMetersPerSecond * RobotConfig.lookaheadSeconds,
                speeds.omegaRadiansPerSecond * RobotConfig.lookaheadSeconds));
    var entry = new Pose3d(future).transformBy(RobotConfig.robotToTurret);
    var target = MiscUtil.AllianceFlip.apply(FieldConstants.Hub.topCenterPoint);
    var offset = future.getTranslation().minus(target.toTranslation2d());
    // interpolate 20inches closer on xy plane
    offset = offset.div(offset.getNorm()).times(Units.inchesToMeters(20));
    // target = target.plus(new Translation3d(offset));

    var shotVector = target.minus(entry.getTranslation());

    double distance = shotVector.getNorm();
    double exitSpeed = distanceToTrajectorySpeed(distance);

    shotVector =
        shotVector
            .div(distance)
            .times(exitSpeed)
            .minus(new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0));

    Rotation2d yaw = shotVector.toTranslation2d().getAngle();
    double relativeHeight = target.getZ() - entry.getZ();
    var pitch = getShotPitch(shotVector.getNorm(), distance, relativeHeight, 0.5, 2.5);

    Logger.recordOutput("RobotState/Turret/overallSpeedRequested", exitSpeed);
    Logger.recordOutput("RobotState/Turret/projectileStart", entry);
    Logger.recordOutput("RobotState/Turret/target", target);
    Logger.recordOutput("RobotState/Turret/relativeHeight", relativeHeight);
    Logger.recordOutput("RobotState/Turret/hubDistance", distance);
    Logger.recordOutput("RobotState/Turret/turretSpeed", shotVector.getNorm());
    Logger.recordOutput("RobotState/Turret/outputYaw", yaw);
    Logger.recordOutput("RobotState/Turret/outputPitchDegs", pitch.getDegrees());

    return new OptimalShot(yaw, pitch, shotVector.getNorm());
  }

  public record OptimalShot(Rotation2d turretYaw, Rotation2d turretPitch, double turretVel) {}

  public record AprilTagObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  public record OdometryObservation(
      ChassisSpeeds speeds,
      SwerveModulePosition[] modulePositions,
      Rotation2d gyroYaw,
      double timestamp) {}
}
