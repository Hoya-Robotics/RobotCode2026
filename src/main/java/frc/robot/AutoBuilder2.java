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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoBuilder2 {
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
  private List<Trajectory<SwerveSample>> trajectories = new ArrayList<>();
  private List<SuperStructureState> superStates = new ArrayList<>();
  private List<TurretTarget> turretTargets = new ArrayList<>();
  private List<Double> delays = new ArrayList<>();
  private List<Command> miscCommands = new ArrayList<>();

  private List<NodeType> graph = new ArrayList<>();

  private boolean shouldFlipYAxis;

  public AutoBuilder2(boolean shouldFlipYAxis) {
    this.shouldFlipYAxis = shouldFlipYAxis;
  }

  private Pose2d flipPoseYAxis(Pose2d pose) {
    return shouldFlipYAxis
        ? new Pose2d(
            new Translation2d(pose.getX(), FieldConstants.fieldWidth - pose.getY()),
            pose.getRotation().unaryMinus())
        : pose;
  }

  private String flippedTraj(String name) {
    return shouldFlipYAxis ? name + "Left" : name;
  }

  public Command generate(Drive drive, SuperStructure superStructure) {
    List<Command> commands = new ArrayList<>();
    int[] nodeFreqs = new int[NodeType.values().length];
    commands.add(
        Commands.runOnce(
            () -> {
              RobotState.getInstance()
                  .resetOdometry(
                      trajectories.get(0).getInitialPose(!FieldConstants.isBlueAlliance()).get());
              autoTimer.restart();
            }));
    int n = 0;
    for (NodeType node : graph) {
      int ctxIndex = nodeFreqs[node.ordinal()];
      commands.add(
          switch (node) {
            case WAIT -> Commands.waitSeconds(delays.get(ctxIndex));
            case DRIVE_TO_POSE ->
                drive.driveToPoseCommandDeferred(
                    () -> flipPoseYAxis(targetPoses.get(ctxIndex).get()));
            case TARGET_TRACK -> superStructure.setTarget(turretTargets.get(ctxIndex));
            case STATE_CHANGE -> superStructure.setStateCommand(superStates.get(ctxIndex));
            case CHOREO_TRAJ -> drive.followChoreoTrajectoryCommand(trajectories.get(ctxIndex));
            case COMMAND -> miscCommands.get(ctxIndex);
            case CAPTURE_REWIND ->
                Commands.runOnce(() -> RobotState.getInstance().captureRewind(autoTimer.get()));
          });
      String key = "Auto/" + n + "-" + node.name();
      commands.add(Commands.runOnce(() -> Logger.recordOutput(key, autoTimer.get())));
      nodeFreqs[node.ordinal()] += 1;
      n += 1;
    }
    return Commands.sequence(commands.toArray(Command[]::new));
  }

  private AutoBuilder2 append(AutoBuilder2 other) {
    targetPoses.addAll(other.targetPoses);
    miscCommands.addAll(other.miscCommands);
    superStates.addAll(other.superStates);
    delays.addAll(other.delays);
    turretTargets.addAll(other.turretTargets);
    trajectories.addAll(other.trajectories);
    graph.addAll(other.graph);
    return this;
  }

  public AutoBuilder2 join(AutoBuilder2 other) {
    return this.copy().append(other);
  }

  public AutoBuilder2 copy() {
    return new AutoBuilder2(shouldFlipYAxis).append(this);
  }

  public AutoBuilder2 captureRewind() {
    var copy = this.copy();
    copy.graph.add(NodeType.CAPTURE_REWIND);
    return copy;
  }

  public AutoBuilder2 withCommand(Command command) {
    var copy = this.copy();
    copy.graph.add(NodeType.COMMAND);
    copy.miscCommands.add(command);
    return copy;
  }

  public AutoBuilder2 withStateChange(SuperStructureState state) {
    var copy = this.copy();
    copy.graph.add(NodeType.STATE_CHANGE);
    copy.superStates.add(state);
    return copy;
  }

  public AutoBuilder2 withTrackTarget(TurretTarget target) {
    var copy = this.copy();
    copy.graph.add(NodeType.TARGET_TRACK);
    copy.turretTargets.add(target);
    return copy;
  }

  public AutoBuilder2 withDelay(double seconds) {
    var copy = this.copy();
    copy.graph.add(NodeType.WAIT);
    copy.delays.add(seconds);
    return copy;
  }

  public AutoBuilder2 withDelayTillRemaining(double secondsRemaining) {
    var copy = this.copy();
    copy.graph.add(NodeType.COMMAND);
    copy.miscCommands.add(
        Commands.defer(
            () -> Commands.waitSeconds(20.0 - secondsRemaining - autoTimer.get()), Set.of()));
    return copy;
  }

  public AutoBuilder2 withDriveToPose(Supplier<Pose2d> pose) {
    var copy = this.copy();
    copy.graph.add(NodeType.DRIVE_TO_POSE);
    copy.targetPoses.add(pose);
    return copy;
  }

  public AutoBuilder2 withDriveToPoseAllianceAgnostic(Pose2d pose) {
    return withDriveToPose(() -> AllianceFlip.apply(pose));
  }

  public AutoBuilder2 withChoreoTraj(String name) {
    var copy = this.copy();
    copy.graph.add(NodeType.CHOREO_TRAJ);
    copy.trajectories.add(
        (Trajectory<SwerveSample>) Choreo.loadTrajectory(flippedTraj(name)).get());
    System.out.println("Loaded choreo traj:" + flippedTraj(name));
    return copy;
  }

  private static AutoBuilder2 swipeTemplate(
      String trajName, boolean endsIntakeToNeutral, boolean shouldFlipYAxis) {
    return new AutoBuilder2(shouldFlipYAxis)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj(trajName)
        .withStateChange(SuperStructureState.SHOOT)
        .withDriveToPoseAllianceAgnostic(
            new Pose2d(3.5784, 0.663, endsIntakeToNeutral ? Rotation2d.kZero : Rotation2d.k180deg));
  }

  private static AutoBuilder2 cleanSwipeTemplate(boolean shouldFlipYAxis) {
    return swipeTemplate("CleanSwipe", false, shouldFlipYAxis);
  }

  private static AutoBuilder2 fullSwipeTemplate(boolean shouldFlipYAxis) {
    return swipeTemplate("FullFuelSwipe", true, shouldFlipYAxis);
  }

  public static Command doubleSwipe(
      Drive drive, SuperStructure superStructure, boolean shouldFlipYAxis) {
    return fullSwipeTemplate(shouldFlipYAxis)
        .withDelay(3.5)
        .join(cleanSwipeTemplate(shouldFlipYAxis))
        .generate(drive, superStructure);
  }

  /*public static Command doubleSwipeBumpExit(
      Drive drive, SuperStructure superStructure, boolean shouldFlipYAxis) {
    return new AutoBuilder2(shouldFlipYAxis)
    .withStateChange(SuperStructureState.INTAKE)
    .withChoreoTraj("FullFuelSwipeBump")
    .withStateChange(SuperStructureState.SHOOT)
  }*/

  public static Command doubleSwipeOutpost(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate(false)
        .withDelay(2.75)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CleanSwipe")
        .withStateChange(SuperStructureState.SHOOT)
        .withDriveToPose(FieldConstants::getHumanStation)
        .withDelayTillRemaining(1.25)
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("LeaveOutpost")
        .generate(drive, superStructure);
  }

  public static Command swipeOutpost(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate(false)
        .withDelay(2.75)
        .withDriveToPose(FieldConstants::getHumanStation)
        .withDelayTillRemaining(0.75)
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("LeaveOutpost")
        .generate(drive, superStructure);
  }

  public static Command doubleSwipeDepot(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate(true)
        .withDelay(2.5)
        .join(cleanSwipeTemplate(true))
        .withDelay(1.25)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("DepotCycle180")
        .withStateChange(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static Command swipeAndDepot(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate(true)
        .withDelay(3.0)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("DepotCycle")
        .withStateChange(SuperStructureState.SHOOT)
        .withDelayTillRemaining(0.75)
        // .captureRewind()
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("ExitDepot")
        .generate(drive, superStructure);
  }

  public static Command centerDepot(Drive drive, SuperStructure superStructure) {
    return new AutoBuilder2(true)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CenterDepot")
        .withStateChange(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static Command centerOutpost(Drive drive, SuperStructure superStructure) {
    return new AutoBuilder2(true)
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("CenterOutpost")
        .withStateChange(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static Command hailMary(
      Drive drive, SuperStructure superStructure, boolean shouldFlipYAxis) {
    return new AutoBuilder2(shouldFlipYAxis)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("SplitTheGap")
        .withCommand(Commands.runOnce(() -> drive.setIdle()))
        .withStateChange(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static void registerAutoChoices(Drive drive, SuperStructure superStructure) {
    autoChooser.addDefaultOption("2xR", doubleSwipe(drive, superStructure, false));
    autoChooser.addOption("2xL", doubleSwipe(drive, superStructure, true));
    autoChooser.addOption("0xOutpost", centerOutpost(drive, superStructure));
    autoChooser.addOption("1xOutpost", swipeOutpost(drive, superStructure));
    autoChooser.addOption("2xOutpost", doubleSwipeOutpost(drive, superStructure));
    autoChooser.addOption("0xDepot", centerDepot(drive, superStructure));
    autoChooser.addOption("1xDepot", swipeAndDepot(drive, superStructure));
    autoChooser.addOption("2xDepot", doubleSwipeDepot(drive, superStructure));
    autoChooser.addOption("SplitTheGapRight", hailMary(drive, superStructure, false));
    autoChooser.addOption("SplitTheGapLeft", hailMary(drive, superStructure, true));
  }
}
