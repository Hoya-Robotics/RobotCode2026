package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.*;
import frc.robot.subsystems.drive.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.photonvision.PhotonUtils;

public class RobotState {
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(RobotConfig.moduleTranslations);
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Pose2d estimatedPose = Pose2d.kZero;
  private Pose2d odometryPose = Pose2d.kZero;
  private ChassisSpeeds odometrySpeeds = new ChassisSpeeds();
  private Pose3d robotToHub;
  private Supplier<Pose2d> simulatedDrivePoseSupplier;

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {
    AutoLogOutputManager.addObject(this);
  }

  public void addSimPoseSupplier(Supplier<Pose2d> supplier) {
    simulatedDrivePoseSupplier = supplier;
  }

  public void hardSetOdometry(Pose2d pose) {
    this.odometryPose = pose;
  }

  public ChassisSpeeds getChassisVelocity() {
    return odometrySpeeds;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(odometrySpeeds, odometryPose.getRotation());
  }

  @AutoLogOutput(key = "RobotState/hubLocalizedPose")
  public Pose3d getHubLocalizedRobotPose() {
    return robotToHub;
  }

  @AutoLogOutput(key = "RobotState/odometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  public Pose2d getSimulatedDrivePose() {
    return simulatedDrivePoseSupplier.get();
  }

  public void addOdometryObservation(OdometryObservation observation) {
    var twist = kinematics.toTwist2d(lastModulePositions, observation.modulePositions());
    lastModulePositions = observation.modulePositions();

    odometrySpeeds = observation.speeds();
    odometryPose = new Pose2d(odometryPose.exp(twist).getTranslation(), observation.gyroYaw());
  }

  public void addHubObservation(HubObservation observation) {
    FieldConstants.aprilLayout
        .getTagPose(observation.tid)
        .ifPresent(
            (fieldRelTagPose) -> {
              robotToHub =
                  PhotonUtils.estimateFieldToRobotAprilTag(
                      observation.cameraToTarget(),
                      fieldRelTagPose,
                      observation.robotToCamera().inverse());
            });
  }

  public void addVisionObservation(AprilTagObservation observation) {}

  // Kalman probably unecessary for hub? Just use latest?
  // Maybe reject using an ambiguity threshold
  public record HubObservation(Transform3d robotToCamera, Transform3d cameraToTarget, int tid) {}

  public record AprilTagObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  public record OdometryObservation(
      ChassisSpeeds speeds,
      SwerveModulePosition[] modulePositions,
      Rotation2d gyroYaw,
      double timestamp) {}
}
