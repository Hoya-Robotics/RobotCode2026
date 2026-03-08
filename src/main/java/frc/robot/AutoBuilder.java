package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AutoBuilder {
  public interface AutoWaypoint {
    Pose2d getTarget();

    Command getCommand(Drive drive, SuperStructure superStructure);
  }

  public record StopWaypoint(Pose2d pose, SuperStructureState superState, BooleanSupplier finished)
      implements AutoWaypoint {
    @Override
    public Pose2d getTarget() {
      return pose;
    }

    @Override
    public Command getCommand(Drive drive, SuperStructure superStructure) {
      return Commands.runOnce(
              () -> {
                drive.driveToPose(pose);
                superStructure.setState(superState);
              })
          .andThen(
              Commands.waitUntil(() -> drive.atDriveToPoseSetpoint() && finished.getAsBoolean()));
    }
  }

  public record ThroughWaypoint(Pose2d pose, SuperStructureState superState, double proximityMeters)
      implements AutoWaypoint {
    @Override
    public Pose2d getTarget() {
      return pose;
    }

    @Override
    public Command getCommand(Drive drive, SuperStructure superStructure) {
      return Commands.runOnce(
              () -> {
                drive.driveToPose(pose);
                superStructure.setState(superState);
              })
          .andThen(
              Commands.waitUntil(
                  () -> {
                    Pose2d current = RobotState.getInstance().getEstimatedPose();
                    double distance = current.getTranslation().getDistance(pose.getTranslation());
                    return distance < proximityMeters;
                  }));
    }
  }

  /** Hybrid waypoint - blends velocity toward lookahead, ends when reaching lookahead pose. */
  public record HybridWaypoint(
      Pose2d pose,
      Pose2d lookahead,
      SuperStructureState superState,
      double blendStartMeters,
      double proximityMeters)
      implements AutoWaypoint {
    @Override
    public Pose2d getTarget() {
      return pose;
    }

    @Override
    public Command getCommand(Drive drive, SuperStructure superStructure) {
      return Commands.runOnce(
              () -> {
                drive.driveToPoseWithLookahead(pose, lookahead, blendStartMeters);
                superStructure.setState(superState);
              })
          .andThen(
              Commands.waitUntil(
                  () ->
                      RobotState.getInstance()
                              .getEstimatedPose()
                              .getTranslation()
                              .getDistance(lookahead.getTranslation())
                          < proximityMeters));
    }
  }

  public static Command getTrajectoryCommand(
      List<AutoWaypoint> traj, Drive drive, SuperStructure superStructure) {
    return Commands.runOnce(() -> RobotState.getInstance().resetOdometry(traj.get(0).getTarget()))
        .andThen(
            Commands.sequence(
                traj.stream()
                    .map(w -> w.getCommand(drive, superStructure))
                    .toArray(Command[]::new)));
  }

  public static final List<AutoWaypoint> testWaypoints = new ArrayList<>();

  static {
    testWaypoints.add(
        new StopWaypoint(
            new Pose2d(3.633, 0.615, Rotation2d.fromRadians(1.567)),
            SuperStructureState.IDLE,
            () -> true));
    testWaypoints.add(
        new ThroughWaypoint(
            new Pose2d(7.854, 0.615, Rotation2d.fromRadians(1.567)),
            SuperStructureState.IDLE,
            Units.inchesToMeters(45)));
    testWaypoints.add(
        new HybridWaypoint(
            new Pose2d(8.31, 2.89, Rotation2d.fromRadians(1.567)),
            new Pose2d(6.72, 1.615, Rotation2d.fromRadians(1.567)), // Lookahead to next
            SuperStructureState.INTAKE,
            Units.inchesToMeters(72), // Start blending 6 feet out
            Units.inchesToMeters(18)));
    testWaypoints.add(
        new ThroughWaypoint(
            new Pose2d(5.733, 0.615, Rotation2d.fromRadians(1.567)),
            SuperStructureState.IDLE,
            Units.inchesToMeters(20)));
    testWaypoints.add(
        new StopWaypoint(
            new Pose2d(3.633, 0.615, Rotation2d.fromRadians(1.567)),
            SuperStructureState.IDLE,
            () -> true));
  }
}
