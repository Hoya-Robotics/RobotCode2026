package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.util.HashMap;

public class AutoBuilder {
  private static final HashMap<String, Trajectory<SwerveSample>> autoTrajectories = new HashMap<>();

  static {
    String[] trajNames = {"FuelSwipe", "ExitSwipe", "FullFuelSwipe", "FullSecondSwipe"};

    for (String name : trajNames) {
      autoTrajectories.put(name, (Trajectory<SwerveSample>) Choreo.loadTrajectory(name).get());
    }
  }

  public static Command doubleSwipe(Drive drive, SuperStructure superStructure) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .resetOdometry(
                        autoTrajectories
                            .get("FuelSwipe")
                            .getInitialPose(!FieldConstants.isBlueAlliance())
                            .get())),
        superStructure.intake(),
        // superStructure.setTarget(TurretTarget.NEAREST_TAG),
        drive.followChoreoTrajectoryCommand(autoTrajectories.get("FuelSwipe")),
        superStructure.idle(),
        drive.followChoreoTrajectoryCommand(autoTrajectories.get("ExitSwipe")),
        drive.driveToPoseCommand(AllianceFlip.apply(new Pose2d(3.5784, 0.663, Rotation2d.kZero))),
        Commands.runOnce(() -> drive.setIdle()),
        // superStructure.setTarget(TurretTarget.HUB),
        superStructure.shoot(),
        Commands.waitSeconds(3.0),
        superStructure.idle());

    /*
    Pose2d nearTrenchPose = AllianceFlip.apply(new Pose2d(3.5784, 0.663, Rotation2d.kZero));
      return Commands.sequence(
        Commands.runOnce(() -> RobotState.getInstance().resetOdometry(
          autoTrajectories.get("FullFuelSwipe").getInitialPose(! FieldConstants.isBlueAlliance()).get()
        )),
          superStructure.intake(),
          // superStructure.setTarget(TurretTarget.NEAREST_TAG),
          drive.followChoreoTrajectoryCommand(autoTrajectories.get("FullFuelSwipe")),
          superStructure.idle(),
        drive.driveToPoseCommand(nearTrenchPose),
        Commands.runOnce(() -> drive.setIdle()),
          // superStructure.setTarget(TurretTarget.HUB),
          superStructure.shoot(),
        Commands.waitSeconds(3.0),
          // superStructure.setTarget(TurretTarget.NEAREST_TAG),
          drive.followChoreoTrajectoryCommand(autoTrajectories.get("SecondFuelSwipe")),
          superStructure.idle(),
        drive.driveToPoseCommand(nearTrenchPose),
        Commands.runOnce(() -> drive.setIdle()),
          // superStructure.setTarget(TurretTarget.HUB),
          superStructure.shoot(),
        Commands.waitSeconds(3.0),
          superStructure.idle()
    );*/
  }
}
