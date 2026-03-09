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
import org.littletonrobotics.junction.Logger;

public class AutoBuilder {
  public interface AutoWaypoint {
    Pose2d getTarget();

    Command getCommand(Drive drive, SuperStructure superStructure);
  }

  public record StopWaypoint(
      Pose2d pose,
      SuperStructureState throughState,
      SuperStructureState stoppedState,
      BooleanSupplier finished)
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
                superStructure.setState(throughState);
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
                Logger.recordOutput("AutoBuilder/superState", superState);
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
    Pose2d neutralTrench = new Pose2d(4.389, 0.615, Rotation2d.kZero);

    AutoWaypoint wp1 =
        new ThroughWaypoint(
            new Pose2d(7.854, 0.615, Rotation2d.kZero),
            SuperStructureState.INTAKE,
            Units.inchesToMeters(55));
    AutoWaypoint wp2 =
        new ThroughWaypoint(
            new Pose2d(5.733, 0.615, Rotation2d.kZero),
            SuperStructureState.INTAKE,
            Units.inchesToMeters(30));
    AutoWaypoint fuelPoint =
        new ThroughWaypoint(
            new Pose2d(8.31, 3.89, Rotation2d.fromRadians(1.26)),
            SuperStructureState.INTAKE,
            Units.inchesToMeters(65));

    testWaypoints.add(
        new StopWaypoint(
            neutralTrench, SuperStructureState.IDLE, SuperStructureState.IDLE, () -> true));
    testWaypoints.add(wp1);
    testWaypoints.add(fuelPoint);
    testWaypoints.add(wp2);
    testWaypoints.add(
        new StopWaypoint(
            neutralTrench, SuperStructureState.IDLE, SuperStructureState.IDLE, () -> true));
    testWaypoints.add(wp1);
    testWaypoints.add(fuelPoint);
    testWaypoints.add(wp2);
    testWaypoints.add(
        new StopWaypoint(
            neutralTrench, SuperStructureState.IDLE, SuperStructureState.IDLE, () -> true));
  }
}
