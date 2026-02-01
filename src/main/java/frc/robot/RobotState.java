package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  private static boolean validTrajectory(Translation3d v, double x) {
    double t = x / Math.hypot(v.getX(), v.getY());
    double y = v.getZ() * t - 0.5 * G * t * t;
    double clearance = RobotConfig.hubFunnelClearance.in(Meters) - RobotConfig.robotToTurret.getZ();

    return y > clearance;
  }

  private static double G = 9.8062;

  public OptimalShot getOptimalShot() {
    var robot = odometryPose;
    var v_robot = odometrySpeeds;
    var v_robot_field = getFieldVelocity();

    // 1. Project robot into future (latency comp)
    var future =
        robot.exp(
            new Twist2d(
                v_robot.vxMetersPerSecond * RobotConfig.lookaheadSeconds,
                v_robot.vyMetersPerSecond * RobotConfig.lookaheadSeconds,
                v_robot.omegaRadiansPerSecond * RobotConfig.lookaheadSeconds));
    var turretPos = new Pose3d(future).transformBy(RobotConfig.robotToTurret);
    var target = MiscUtil.AllianceFlip.apply(FieldConstants.Hub.topCenterPoint);

    // 2. Calculate field relative direction vector
    Translation3d trajectoryVector = target.minus(turretPos.getTranslation());

    // 3. Calculate field relative turret velocity
    // 	v_t = v_r(field) + w_r(field) x r_t(field)
    var turret_radius_perp =
        new Translation2d(-RobotConfig.robotToTurret.getY(), RobotConfig.robotToTurret.getX())
            .rotateBy(future.getRotation());

    var v_turret_field =
        new Translation2d(v_robot_field.vxMetersPerSecond, v_robot_field.vyMetersPerSecond)
            .plus(turret_radius_perp.times(v_robot_field.omegaRadiansPerSecond));

    double funnelHorizontalDistance =
        MiscUtil.AllianceFlip.apply(FieldConstants.Hub.nearFace)
            .getTranslation()
            .getDistance(future.getTranslation());

    double t_min = 0.0;
    double t_max = 6.0;
    int samples = 100;

    double bestCost = Double.MAX_VALUE;
    var shot = new OptimalShot(Rotation2d.kZero, Rotation2d.kZero, 0.0);
    double clearance = 0.0;
    for (int i = 0; i < samples; i++) {
      double t = t_min + i * (t_max - t_min) / (samples - 1);
      var v_fuel_field =
          new Translation3d(
              trajectoryVector.getX() / t,
              trajectoryVector.getY() / t,
              (trajectoryVector.getZ() + 0.5 * G * t * t) / t);
      var v_fuel_rel = v_fuel_field.minus(new Translation3d(v_turret_field));
      var pitch = Math.atan2(v_fuel_rel.getZ(), Math.hypot(v_fuel_rel.getY(), v_fuel_rel.getX()));
      var yaw = v_fuel_rel.toTranslation2d().getAngle();
      var parameterVector =
          VecBuilder.fill(
              v_fuel_rel.getSquaredNorm(),
              Math.pow(pitch - RobotConfig.optimalPitch.in(Radians), 2),
              t);
      double cost = parameterVector.dot(RobotConfig.trajectoryWeights);
      if (validTrajectory(v_fuel_field, funnelHorizontalDistance) && cost < bestCost) {
        bestCost = cost;
        shot = new OptimalShot(yaw, Rotation2d.fromRadians(pitch), v_fuel_rel.getNorm());
        {
          double tt = funnelHorizontalDistance / v_fuel_field.toTranslation2d().getNorm();
          double y =
              RobotConfig.robotToTurret.getZ() + v_fuel_field.getZ() * tt - 0.5 * G * tt * tt;
          clearance = Units.metersToInches(y - RobotConfig.hubFunnelClearance.in(Meters));
        }
      }
    }
    Logger.recordOutput("OptimalShot/velocity", shot.turretVel());
    Logger.recordOutput("OptimalShot/pitch", shot.turretPitch());
    Logger.recordOutput("OptimalShot/yaw", shot.turretYaw());
    Logger.recordOutput("OptimalShot/clearanceInches", clearance);

    return shot;
  }

  public record OptimalShot(Rotation2d turretYaw, Rotation2d turretPitch, double turretVel) {}

  public record AprilTagObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  public record OdometryObservation(
      ChassisSpeeds speeds,
      SwerveModulePosition[] modulePositions,
      Rotation2d gyroYaw,
      double timestamp) {}
}
