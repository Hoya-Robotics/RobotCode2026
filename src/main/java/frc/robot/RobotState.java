package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
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

  private static double G = 9.80665;

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

    // NOTE: From here on object creation is minimized for sampling performance
    double targetX = trajectoryVector.getX();
    double targetY = trajectoryVector.getY();
    double targetZ = trajectoryVector.getZ();

    double v_turret_x = v_turret_field.getX();
    double v_turret_y = v_turret_field.getY();
    double v_turret_z = 0.0;

    double t_min = 0.0;
    double t_max = 6.0;
    int samples = 100;

    double optimalPitch = RobotConfig.optimalPitch.in(Radians);
    double bestCost = Double.MAX_VALUE;
    var shot = new OptimalShot(Rotation2d.kZero, Rotation2d.kZero, 0.0);
    double clearance = RobotConfig.hubFunnelClearance.in(Meters);

    for (int i = 0; i < samples; i++) {
      double t = t_min + i * (t_max - t_min) / (samples - 1);

      double v_field_x = targetX / t;
      double v_field_y = targetY / t;
      double v_field_z = (targetZ + 0.5 * G * t * t) / t;

      double v_rel_x = v_field_x - v_turret_x;
      double v_rel_y = v_field_y - v_turret_y;
      double v_rel_z = v_field_z - v_turret_z;

      double v_hypot_xy = Math.hypot(v_rel_x, v_rel_y);
      double pitch_rads = Math.atan2(v_rel_z, v_hypot_xy);
      double v = Math.hypot(v_rel_z, v_hypot_xy);

      double vCost = v * v;
      double pitchCost = Math.pow(pitch_rads - optimalPitch, 2);
      double tCost = t;

      double cost =
          (vCost * RobotConfig.trajectoryWeights.get(0, 0))
              + (pitchCost * RobotConfig.trajectoryWeights.get(1, 0))
              + (tCost * RobotConfig.trajectoryWeights.get(2, 0));

      double t_funnel = funnelHorizontalDistance / Math.hypot(v_field_x, v_field_y);
      double y_funnel = v_field_z * t_funnel - 0.5 * G * t_funnel * t_funnel;

      if (y_funnel > clearance && cost < bestCost) {
        bestCost = cost;
        shot = new OptimalShot(new Rotation2d(v_rel_x, v_rel_y), new Rotation2d(pitch_rads), v);
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
