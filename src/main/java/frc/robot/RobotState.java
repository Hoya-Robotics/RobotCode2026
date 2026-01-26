package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.*;

public class RobotState {
  private final double poseBufferSizeSec = 2.0;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] modulePositions;

  private Pose2d odometryPose = Pose2d.kZero;
  private Pose2d estimatedPose = Pose2d.kZero;

  private static RobotState instance;

  // Singleton pattern
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {
    kinematics = new SwerveDriveKinematics(DriveConstants.modulePositions);
    modulePositions = new SwerveModulePosition[4];
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(modulePositions, observation.modulePositions());
    modulePositions = observation.modulePositions();
    Pose2d lastPose = odometryPose;
    odometryPose = new Pose2d(odometryPose.exp(twist).getTranslation(), observation.gyroAngle());

    poseBuffer.addSample(observation.timestamp(), odometryPose);

    estimatedPose = estimatedPose.exp(lastPose.log(odometryPose));
  }

  public void addVisionObservation(VisionObservation observation) {}

  public record VisionObservation() {}

  public record OdometryObservation(
      double timestamp, SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {}
}
