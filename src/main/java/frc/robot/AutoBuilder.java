package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AutoBuilder {
  public record AutoWaypoint(
      Pose2d pose, SuperStructureState superState, BooleanSupplier finished) {
    public Command getCommand(Drive drive, SuperStructure superStructure) {
      return Commands.runOnce(
              () -> {
                drive.driveToPose(pose);
                superStructure.setState(superState);
              })
          .until(() -> drive.atDriveToPoseSetpoint() && finished.getAsBoolean());
    }

    public static Command getTrajectoryCommand(
        List<AutoWaypoint> traj, Drive drive, SuperStructure superStructure) {
      return Commands.sequence(
          traj.stream().map(w -> w.getCommand(drive, superStructure)).toArray(Command[]::new));
    }
  }

  public static final List<AutoWaypoint> testWaypoints = new ArrayList<>();

  static {
    testWaypoints.add(
        new AutoWaypoint(
            new Pose2d(3.633, 0.615, Rotation2d.fromRadians(1.567)),
            SuperStructureState.IDLE,
            () -> true));
    testWaypoints.add(
        new AutoWaypoint(
            new Pose2d(7.854, 0.615, Rotation2d.fromRadians(1.567)),
            SuperStructureState.INTAKE,
            () -> true));
  }
}
