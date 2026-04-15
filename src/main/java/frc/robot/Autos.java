package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.DriveConstants;
import frc.robot.RobotConfig.IntakeConstants.IntakeState;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlip;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private static LoggedDashboardChooser<Command> dashboardChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private static AutoFactory choreoFactory;
  private static Timer autoTimer = new Timer();

  private static final List<Pair<String, Double>> pathPlannerAutos =
      List.of(Pair.of("BumpAuto", 0.0), Pair.of("DoubleSwipe", 4.0));

  public static void warmup(Drive drive, SuperStructure superStructure, Intake intake) {
    com.pathplanner.lib.config.RobotConfig pp_config = null;
    try {
      pp_config = com.pathplanner.lib.config.RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          RobotState.getInstance()::getEstimatedPose,
          RobotState.getInstance()::resetOdometry,
          RobotState.getInstance()::getRobotVelocity,
          drive::followPathplannerTrajectory,
          new PPHolonomicDriveController(
              DriveConstants.PP_translationConstants, DriveConstants.PP_rotationConstants),
          pp_config,
          () -> !FieldConstants.isBlueAlliance(),
          drive);
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Schedule warmup commands first so they get maximum scheduler cycles during disabled
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    choreoFactory =
        new AutoFactory(
            RobotState.getInstance()::getEstimatedPose,
            RobotState.getInstance()::resetOdometry,
            drive::followChoreoTrajectory,
            true,
            drive);
    CommandScheduler.getInstance().schedule(choreoFactory.warmupCmd());

    dashboardChooser.addOption("2xSwipe |R", doubleSwipe(drive, superStructure, intake, false));
    dashboardChooser.addOption("2xSwipe |L", doubleSwipe(drive, superStructure, intake, true));
    dashboardChooser.addOption("2xBump  |R", doubleBumpSwipe(drive, superStructure, intake, false));
    dashboardChooser.addOption("2xBump  |L", doubleBumpSwipe(drive, superStructure, intake, true));
    dashboardChooser.addOption("Depot   |C", centerDepot(drive, superStructure, intake));

    // Generate pathplanner autos with mirrored version
    for (var entry : pathPlannerAutos) {
      String name = entry.getFirst();
      double delay = entry.getSecond();

      List<PathPlannerPath> paths, mirroredPaths;
      try {
        paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
      } catch (Exception e) {
        System.out.println("Failed to load pathplanner auto: " + name);
        e.printStackTrace();
        continue;
      }
      mirroredPaths = paths.stream().map(PathPlannerPath::mirrorPath).toList();

      // Eagerly pre-generate ideal trajectories for all path variants (original, mirrored, flipped)
      // to avoid expensive trajectory generation causing a cycle spike on the first auto path
      if (pp_config != null) {
        for (PathPlannerPath path : paths) {
          path.getIdealTrajectory(pp_config);
          path.flipPath().getIdealTrajectory(pp_config);
        }
        for (PathPlannerPath path : mirroredPaths) {
          path.getIdealTrajectory(pp_config);
          path.flipPath().getIdealTrajectory(pp_config);
        }
      }

      dashboardChooser.addOption(
          name + "|R",
          wrapShootAllianceIntakeNeutral(
              followPathplannerPathGroup(paths, delay), superStructure, intake));
      dashboardChooser.addOption(
          name + "|L",
          wrapShootAllianceIntakeNeutral(
              followPathplannerPathGroup(mirroredPaths, delay), superStructure, intake));
    }
  }

  private static Command followPathplannerPathGroup(
      List<PathPlannerPath> paths, double delaysBetween) {
    return AutoBuilder.resetOdom(paths.get(0).getStartingHolonomicPose().get())
        .andThen(
            Commands.sequence(
                paths.stream()
                    .map(
                        p -> AutoBuilder.followPath(p).andThen(Commands.waitSeconds(delaysBetween)))
                    .toArray(Command[]::new)));
  }

  public static Command getAuto() {
    return dashboardChooser.get();
  }

  private static Command flippableTrajectory(String trajName, boolean mirrorYAxis) {
    return mirrorYAxis
        ? choreoFactory.trajectoryCmd(trajName, AutoTrajectory::mirrorY)
        : choreoFactory.trajectoryCmd(trajName);
  }

  private static Translation2d rightTrenchShoootTranslation = new Translation2d(3.4, 0.665);

  private static Command alignToTrench(Drive drive, Rotation2d heading, boolean mirrorY) {
    Pose2d mirrored =
        new Pose2d(
            new Translation2d(
                rightTrenchShoootTranslation.getMeasureX(),
                FieldConstants.fieldWidth.minus(rightTrenchShoootTranslation.getMeasureY())),
            heading.unaryMinus());
    return drive.driveToPoseCommandDeferred(
        () ->
            AllianceFlip.apply(
                mirrorY ? mirrored : new Pose2d(rightTrenchShoootTranslation, heading)));
  }

  private static Command wrapShootAllianceIntakeNeutral(
      Command routine, SuperStructure superStructure, Intake intake) {
    return routine.alongWith(
        Commands.either(
                superStructure.setStateCommand(SuperStructureState.PRE_SHOOT),
                superStructure.setStateCommand(SuperStructureState.IDLE),
                () -> FieldConstants.inAllianceZone(RobotState.getInstance().getEstimatedPose()))
            .repeatedly(),
        Commands.either(
                intake.setStateCommand(IntakeState.IDLE),
                intake.setStateCommand(IntakeState.INTAKE),
                () -> FieldConstants.inAllianceZone(RobotState.getInstance().getEstimatedPose()))
            .repeatedly());
  }

  private static Command doubleSwipe(
      Drive drive, SuperStructure superStructure, Intake intake, boolean mirrorYAxis) {
    return wrapShootAllianceIntakeNeutral(
        Commands.sequence(
            Commands.runOnce(() -> autoTimer.start()),
            flippableTrajectory("MogSwipe", mirrorYAxis),
            alignToTrench(drive, Rotation2d.kZero, mirrorYAxis),
            Commands.runOnce(() -> Logger.recordOutput("Auto/firstCycleEnd", autoTimer.get())),
            Commands.waitSeconds(4.2),
            flippableTrajectory("CleanSwipe", mirrorYAxis),
            alignToTrench(drive, Rotation2d.kZero, mirrorYAxis),
            Commands.runOnce(() -> Logger.recordOutput("Auto/secondCycleEnd", autoTimer.get()))),
        superStructure,
        intake);
  }

  private static Command doubleBumpSwipe(
      Drive drive, SuperStructure superStructure, Intake intake, boolean mirrorYAxis) {
    return wrapShootAllianceIntakeNeutral(
        Commands.sequence(
            flippableTrajectory("OPStart", mirrorYAxis),
            flippableTrajectory("OPEnd2", mirrorYAxis),
            flippableTrajectory("OPEscape", mirrorYAxis)),
        superStructure,
        intake);
  }

  private static Command centerDepot(Drive drive, SuperStructure superStructure, Intake intake) {
    return Commands.sequence(
        intake.setStateCommand(IntakeState.INTAKE),
        choreoFactory.trajectoryCmd("CenterDepotCollect"),
        superStructure.setStateCommand(SuperStructureState.SHOOT));
  }
}
