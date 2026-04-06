package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private static LoggedDashboardChooser<Command> dashboardChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private static AutoFactory choreoFactory;

  public static void warmup(Drive drive, SuperStructure superStructure) {
    choreoFactory =
        new AutoFactory(
            RobotState.getInstance()::getEstimatedPose,
            RobotState.getInstance()::resetOdometry,
            drive::followChoreoTrajectory,
            true,
            drive);

    dashboardChooser.addOption("2xSwipe|R", doubleSwipe(drive, superStructure, false));
    dashboardChooser.addOption("2xSwipe|L", doubleSwipe(drive, superStructure, true));
    dashboardChooser.addOption("2xBump|R", doubleBumpSwipe(drive, superStructure, false));
    dashboardChooser.addOption("2xBump|L", doubleBumpSwipe(drive, superStructure, true));
    dashboardChooser.addOption("Depot|C", centerDepot(drive, superStructure));

    CommandScheduler.getInstance().schedule(choreoFactory.warmupCmd());
  }

  public static Command getAuto() {
    return dashboardChooser.get();
  }

  private static Command flippableTrajectory(String trajName, boolean mirrorYAxis) {
    return Commands.either(
        choreoFactory.trajectoryCmd(trajName, AutoTrajectory::mirrorY),
        choreoFactory.trajectoryCmd(trajName),
        () -> mirrorYAxis);
  }

  private static Pose2d rightTrenchShootPose = new Pose2d(3.4, 0.665, Rotation2d.kZero);

  private static Command alignToTrench(Drive drive, boolean mirrorY) {
    Pose2d mirrored =
        new Pose2d(
            new Translation2d(
                rightTrenchShootPose.getMeasureX(),
                FieldConstants.fieldWidth.minus(rightTrenchShootPose.getMeasureY())),
            rightTrenchShootPose.getRotation().unaryMinus());
    return drive.driveToPoseCommandDeferred(
        () -> AllianceFlip.apply(mirrorY ? mirrored : rightTrenchShootPose));
  }

  private static Command wrapShootAllianceIntakeNeutral(
      Command routine, SuperStructure superStructure) {
    return routine.alongWith(
        Commands.run(
            () ->
                superStructure.setState(
                    FieldConstants.inAllianceZone(RobotState.getInstance().getEstimatedPose())
                        ? SuperStructureState.SHOOT
                        : SuperStructureState.INTAKE),
            superStructure));
  }

  private static Command doubleSwipe(
      Drive drive, SuperStructure superStructure, boolean mirrorYAxis) {
    return wrapShootAllianceIntakeNeutral(
        Commands.sequence(
            flippableTrajectory("MogSwipe", mirrorYAxis),
            alignToTrench(drive, mirrorYAxis),
            Commands.waitSeconds(4.2),
            flippableTrajectory("CleanSwipe", mirrorYAxis),
            alignToTrench(drive, mirrorYAxis)),
        superStructure);
  }

  private static Command doubleBumpSwipe(
      Drive drive, SuperStructure superStructure, boolean mirrorYAxis) {
    return wrapShootAllianceIntakeNeutral(
        Commands.sequence(
            flippableTrajectory("OPStart", mirrorYAxis),
            flippableTrajectory("OPEnd2", mirrorYAxis),
            flippableTrajectory("OPEscape", mirrorYAxis)),
        superStructure);
  }

  private static Command centerDepot(Drive drive, SuperStructure superStructure) {
    return Commands.sequence(
        superStructure.setStateCommand(SuperStructureState.INTAKE),
        choreoFactory.trajectoryCmd("CenterDepotCollect"),
        superStructure.setStateCommand(SuperStructureState.SHOOT));
  }
}
