package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance;

  // Singleton pattern
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private Pose2d odometryPose = Pose2d.kZero;
  private Supplier<Pose2d> simulatedPoseCallback = null;

  public void addOdometryObservation(OdometryObservation observation) {
    final var robotDisplacement =
        Arrays.stream(observation.moduleDisplacements)
            .reduce(Translation2d.kZero, Translation2d::plus)
            .rotateBy(observation.gyroYaw);
    final Pose2d poseEstimate =
        new Pose2d(odometryPose.getTranslation().plus(robotDisplacement), observation.gyroYaw);

    odometryPose = poseEstimate;
    Logger.recordOutput("RobotState/odometryPose", odometryPose);
  }

  public void addSimulatedPoseCallback(Supplier<Pose2d> simulatedPoseCallback) {
    this.simulatedPoseCallback = simulatedPoseCallback;
  }

  // TODO: implement
  public void addVisionObservation(AprilTagObservation observation) {}

  public final Pose2d getOdometryPose() {
    return odometryPose;
  }

  public final Pose2d getSimulatedPose() {
    return simulatedPoseCallback.get();
  }

  // TODO: add observation functions for vision and drive odometry
  public Pose2d getEstimatedRobotPose() {
    return Pose2d.kZero;
  }

  public record AprilTagObservation(double timestamp, Pose2d visionPose) {}

  public record OdometryObservation(
      double timestamp, Rotation2d gyroYaw, Translation2d[] moduleDisplacements) {}
}
