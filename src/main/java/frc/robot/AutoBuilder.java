package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoBuilder {
  enum NodeType {
    STATE_CHANGE,
    TARGET_TRACK,
    DRIVE_TO_POSE,
    CHOREO_TRAJ,
    WAIT,
    COMMAND,
    CAPTURE_REWIND
  }

  private static Timer autoTimer = new Timer();
  public static LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private List<Supplier<Pose2d>> targetPoses = new ArrayList<>();
  private List<String> trajectoryNames = new ArrayList<>();
  private List<SuperStructureState> superStates = new ArrayList<>();
  private List<TurretTarget> turretTargets = new ArrayList<>();
  private List<Double> delays = new ArrayList<>();
  private List<Command> miscCommands = new ArrayList<>();

  private List<NodeType> graph = new ArrayList<>();

  public AutoBuilder() {}

  private Pose2d flipPoseYAxis(Pose2d pose, boolean isRightSide) {
    return isRightSide
        ? pose
        : new Pose2d(
            new Translation2d(pose.getX(), FieldConstants.fieldWidth - pose.getY()),
            pose.getRotation().unaryMinus());
  }

  private String flippedTraj(String name, boolean isRightSide) {
    return isRightSide ? name : name + "Left";
  }

  public Command generate(Drive drive, SuperStructure superStructure, boolean isRightSide) {
    List<Command> commands = new ArrayList<>();
    int[] nodeFreqs = new int[NodeType.values().length];
    NodeType firstMove =
        graph.stream()
            .filter(n -> n == NodeType.DRIVE_TO_POSE || n == NodeType.CHOREO_TRAJ)
            .findFirst()
            .get();
    Supplier<Pose2d> initialPose =
        () ->
            switch (firstMove) {
              case DRIVE_TO_POSE -> flipPoseYAxis(targetPoses.get(0).get(), isRightSide);
              case CHOREO_TRAJ ->
                  Choreo.loadTrajectory(flippedTraj(trajectoryNames.get(0), isRightSide))
                      .get()
                      .getInitialPose(!FieldConstants.isBlueAlliance())
                      .get();
              default -> Pose2d.kZero;
            };
    commands.add(
        Commands.defer(
            () -> Commands.runOnce(() -> RobotState.getInstance().resetOdometry(initialPose.get())),
            Set.of()));
    commands.add(Commands.runOnce(() -> autoTimer.restart()));
    for (NodeType node : graph) {
      int ctxIndex = nodeFreqs[node.ordinal()];
      commands.add(
          switch (node) {
            case WAIT -> Commands.waitSeconds(delays.get(ctxIndex));
            case DRIVE_TO_POSE ->
                drive.driveToPoseCommandDeferred(
                    () -> flipPoseYAxis(targetPoses.get(ctxIndex).get(), isRightSide));
            case TARGET_TRACK -> superStructure.setTarget(turretTargets.get(ctxIndex));
            case STATE_CHANGE ->
                Commands.runOnce(() -> superStructure.setState(superStates.get(ctxIndex)));
            case CHOREO_TRAJ ->
                drive.followChoreoTrajectoryCommand(
                    (Trajectory<SwerveSample>)
                        Choreo.loadTrajectory(
                                flippedTraj(trajectoryNames.get(ctxIndex), isRightSide))
                            .get());
            case COMMAND -> miscCommands.get(ctxIndex);
            case CAPTURE_REWIND ->
                Commands.runOnce(
                    () -> {
                      double autoTime = autoTimer.get();
                      RobotState.getInstance().captureRewind(autoTime);
                    });
          });
      nodeFreqs[node.ordinal()] += 1;
    }
    return Commands.sequence(commands.toArray(Command[]::new));
  }

  private AutoBuilder append(AutoBuilder other) {
    targetPoses.addAll(other.targetPoses);
    miscCommands.addAll(other.miscCommands);
    superStates.addAll(other.superStates);
    delays.addAll(other.delays);
    turretTargets.addAll(other.turretTargets);
    trajectoryNames.addAll(other.trajectoryNames);
    graph.addAll(other.graph);
    return this;
  }

  public AutoBuilder join(AutoBuilder other) {
    return this.copy().append(other);
  }

  public AutoBuilder copy() {
    return new AutoBuilder().append(this);
  }

  public AutoBuilder captureRewind() {
    var copy = this.copy();
    copy.graph.add(NodeType.CAPTURE_REWIND);
    return copy;
  }

  public AutoBuilder withCommand(Command command) {
    var copy = this.copy();
    copy.graph.add(NodeType.COMMAND);
    copy.miscCommands.add(command);
    return copy;
  }

  public AutoBuilder withStateChange(SuperStructureState state) {
    var copy = this.copy();
    copy.graph.add(NodeType.STATE_CHANGE);
    copy.superStates.add(state);
    return copy;
  }

  public AutoBuilder withTrackTarget(TurretTarget target) {
    var copy = this.copy();
    copy.graph.add(NodeType.TARGET_TRACK);
    copy.turretTargets.add(target);
    return copy;
  }

  public AutoBuilder withDelay(double seconds) {
    var copy = this.copy();
    copy.graph.add(NodeType.WAIT);
    copy.delays.add(seconds);
    return copy;
  }

  public AutoBuilder withDelayTillRemaining(double secondsRemaining) {
    var copy = this.copy();
    copy.graph.add(NodeType.COMMAND);
    copy.miscCommands.add(
        Commands.defer(
            () -> Commands.waitSeconds(20.0 - secondsRemaining - autoTimer.get()), Set.of()));
    return copy;
  }

  public AutoBuilder withDriveToPose(Supplier<Pose2d> pose) {
    var copy = this.copy();
    copy.graph.add(NodeType.DRIVE_TO_POSE);
    copy.targetPoses.add(pose);
    return copy;
  }

  public AutoBuilder withDriveToPoseAllianceAgnostic(Pose2d pose) {
    return withDriveToPose(() -> AllianceFlip.apply(pose));
  }

  public AutoBuilder withChoreoTraj(String name) {
    var copy = this.copy();
    copy.graph.add(NodeType.CHOREO_TRAJ);
    copy.trajectoryNames.add(name);
    return copy;
  }

  private static AutoBuilder swipeTemplate(String trajName, boolean endsIntakeToNeutral) {
    return new AutoBuilder()
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj(trajName)
        .withStateChange(SuperStructureState.SHOOT)
        .withDriveToPoseAllianceAgnostic(
            new Pose2d(3.5784, 0.663, endsIntakeToNeutral ? Rotation2d.kZero : Rotation2d.k180deg));
  }

  private static AutoBuilder cleanSwipeTemplate = swipeTemplate("CleanSwipe", false);
  private static AutoBuilder fullSwipeTemplate = swipeTemplate("FullFuelSwipe", true);

  public static Command doubleSwipeCleanup(
      Drive drive, SuperStructure superStructure, boolean isRightSide) {
    return fullSwipeTemplate
        .withDelay(3.0)
        .join(cleanSwipeTemplate)
        .withDelayTillRemaining(0.5)
        .captureRewind()
        .withStateChange(SuperStructureState.IDLE)
        .withDriveToPoseAllianceAgnostic(new Pose2d(7.5784, 0.663, Rotation2d.k180deg))
        .generate(drive, superStructure, isRightSide);
  }

  public static Command doubleSwipeCleanupOutpost(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate
        .withDelay(2.75)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CleanSwipe")
        .withStateChange(SuperStructureState.SHOOT)
        .withDriveToPose(FieldConstants::getHumanStation)
        .withDelayTillRemaining(1.25)
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("LeaveOutpost")
        .captureRewind()
        .generate(drive, superStructure, true);
  }

  public static Command swipeAndDepot(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate
        .withDelay(2.0)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("DepotCycle")
        .withStateChange(SuperStructureState.SHOOT)
        .withDelayTillRemaining(1.0)
        .captureRewind()
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("ExitDepot")
        .generate(drive, superStructure, false);
  }

  public static Command shootOnTheMoveTest(Drive drive, SuperStructure superStructure) {
    return new AutoBuilder()
        .withTrackTarget(TurretTarget.ON_THE_MOVE)
        .withStateChange(SuperStructureState.SHOOT)
        .withChoreoTraj("ShootOnTheMoveTest")
        .withStateChange(SuperStructureState.IDLE)
        .captureRewind()
        .generate(drive, superStructure, true);
  }

  public static Command centerDepot(Drive drive, SuperStructure superStructure) {
    return new AutoBuilder()
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CenterDepot")
        .withStateChange(SuperStructureState.SHOOT)
        .generate(drive, superStructure, false);
  }

  public static void registerAutoChoices(Drive drive, SuperStructure superStructure) {
    autoChooser.addDefaultOption("2xR", doubleSwipeCleanup(drive, superStructure, true));
    autoChooser.addOption("2xL", doubleSwipeCleanup(drive, superStructure, false));
    autoChooser.addOption("2xOutpost", doubleSwipeCleanupOutpost(drive, superStructure));
    autoChooser.addOption("0xDepot", centerDepot(drive, superStructure));
    autoChooser.addOption("1xDepot", swipeAndDepot(drive, superStructure));
    autoChooser.addOption("shootOnMoveTest", shootOnTheMoveTest(drive, superStructure));
  }
}
