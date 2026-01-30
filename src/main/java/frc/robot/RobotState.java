package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.*;
import frc.robot.subsystems.drive.*;
import frc.robot.util.MiscUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

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
    return 10 / -Math.exp(distanceMeters - 5);
  }

  // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
  private static final double G = 9.8;

  private static Rotation2d getShotPitch(double v, double x, double y) {
    double top = (v * v) - Math.sqrt(Math.pow(v, 4) - G * (G * x * x + 2 * y * v * v));
    return Rotation2d.fromRadians(Math.atan(top / G * x));
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

    var shotVector = target.minus(entry.getTranslation());
    double distance = shotVector.getNorm();
    shotVector = shotVector.div(distance).times(distanceToTrajectorySpeed(distance));

    Rotation2d yaw = new Rotation2d(shotVector.getX(), shotVector.getY());
    var pitch = getShotPitch(shotVector.getNorm(), distance, target.getZ());
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
