package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.*;
import frc.robot.subsystems.drive.*;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static RobotState instance;

  @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;
  @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(RobotConfig.moduleTranslations);
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // Singleton pattern
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  public void addOdometryObservation(OdometryObservation observation) {
    var twist = kinematics.toTwist2d(lastModulePositions, observation.modulePositions());
    lastModulePositions = observation.modulePositions();

    // var lastPose = odometryPose;
    odometryPose = new Pose2d(odometryPose.exp(twist).getTranslation(), observation.gyroYaw());
  }

  public void addVisionObservation(AprilTagObservation observation) {}

  public record AprilTagObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  public record OdometryObservation(
      SwerveModulePosition[] modulePositions, Rotation2d gyroYaw, double timestamp) {}
}
