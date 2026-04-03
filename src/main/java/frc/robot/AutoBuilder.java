package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoBuilder {
  sealed interface Node permits Node.Wait, Node.DriveToPose, Node.ChoreoTraj, Node.Cmd {
    Command toCommand(Drive drive, SuperStructure superStructure, Timer timer);

    record Wait(double seconds) implements Node {
      public Command toCommand(Drive drive, SuperStructure superStructure, Timer timer) {
        return Commands.waitSeconds(seconds);
      }
    }

    record DriveToPose(Supplier<Pose2d> pose) implements Node {
      public Command toCommand(Drive drive, SuperStructure superStructure, Timer timer) {
        return drive.driveToPoseCommandDeferred(pose);
      }
    }

    record ChoreoTraj(Trajectory<SwerveSample> traj) implements Node {
      public Command toCommand(Drive drive, SuperStructure superStructure, Timer timer) {
        return drive.followChoreoTrajectoryCommand(traj);
      }

      public Command resetOdometry() {
        return Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .resetOdometry(traj().getInitialPose(!FieldConstants.isBlueAlliance()).get()));
      }
    }

    record Cmd(Command command) implements Node {
      public Command toCommand(Drive drive, SuperStructure superStructure, Timer timer) {
        return command;
      }
    }
  }
  ;

  private static Timer autoTimer = new Timer();
  public static LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private List<Node> graph = new ArrayList<>();
  private List<Pair<SuperStructureState, Integer>> stateChanges = new ArrayList<>();
  private Command entireParallelCommand = Commands.none();
  private Optional<SuperStructureState> finalState = Optional.empty();

  private boolean shouldFlipYAxis;

  public AutoBuilder(boolean shouldFlipYAxis) {
    this.shouldFlipYAxis = shouldFlipYAxis;
  }

  private static final Map<String, Trajectory<SwerveSample>> trajCache = new HashMap<>();

  private static Trajectory<SwerveSample> loadTraj(String name) {
    return trajCache.computeIfAbsent(
        name,
        k ->
            (Trajectory<SwerveSample>)
                Choreo.loadTrajectory(k)
                    .orElseThrow(
                        () -> new RuntimeException("Failed to load choreo trajectory: " + k)));
  }

  private Pose2d flipPoseYAxis(Pose2d pose) {
    return shouldFlipYAxis
        ? new Pose2d(
            new Translation2d(
                pose.getMeasureX(), FieldConstants.fieldWidth.minus(pose.getMeasureY())),
            pose.getRotation().unaryMinus())
        : pose;
  }

  private String flippedTraj(String name) {
    return shouldFlipYAxis ? name + "Left" : name;
  }

  public Command generate(Drive drive, SuperStructure superStructure) {
    List<Command> commands = new ArrayList<>();
    commands.add(Commands.runOnce(autoTimer::restart));

    Optional<Node> firstTraj =
        graph.stream().filter(n -> n.getClass().equals(Node.ChoreoTraj.class)).findFirst();
    firstTraj.ifPresent(n -> commands.add(((Node.ChoreoTraj) n).resetOdometry()));

    SuperStructureState currentState = SuperStructureState.IDLE;
    for (int i = 0; i < graph.size(); ++i) {
      final int index = i;
      Node node = graph.get(i);
      var newState = stateChanges.stream().filter(n -> n.getSecond() == index).findFirst();
      if (newState.isPresent()) currentState = newState.get().getFirst();
      commands.add(
          node.toCommand(drive, superStructure, autoTimer)
              .deadlineFor(superStructure.setStateCommand(currentState)));
      String key = "Auto/" + index + "-" + node.getClass().getSimpleName();
      commands.add(Commands.runOnce(() -> Logger.recordOutput(key, autoTimer.get())));
    }

    finalState.ifPresent(s -> commands.add(superStructure.setStateCommand(s).repeatedly()));
    return Commands.sequence(commands.toArray(Command[]::new)).deadlineFor(entireParallelCommand);
  }

  private AutoBuilder parallelAllWith(Command command) {
    this.entireParallelCommand = command;
    return this;
  }

  private AutoBuilder append(AutoBuilder other) {
    int offset = graph.size();
    graph.addAll(other.graph);
    other.stateChanges.forEach(
        p -> stateChanges.add(Pair.of(p.getFirst(), p.getSecond() + offset)));
    other.finalState.ifPresent(s -> this.finalState = Optional.of(s));
    return this;
  }

  public AutoBuilder join(AutoBuilder other) {
    return this.copy().append(other);
  }

  public AutoBuilder copy() {
    return new AutoBuilder(shouldFlipYAxis).append(this);
  }

  public AutoBuilder withCommand(Command command) {
    graph.add(new Node.Cmd(command));
    return this;
  }

  public AutoBuilder withStateChange(SuperStructureState state) {
    stateChanges.add(Pair.of(state, graph.size()));
    return this;
  }

  public AutoBuilder withFinalState(SuperStructureState state) {
    this.finalState = Optional.of(state);
    return this;
  }

  public AutoBuilder withDelay(double seconds) {
    graph.add(new Node.Wait(seconds));
    return this;
  }

  public AutoBuilder withDriveToPose(Supplier<Pose2d> pose) {
    graph.add(new Node.DriveToPose(() -> flipPoseYAxis(pose.get())));
    return this;
  }

  public AutoBuilder withDriveToPoseAllianceAgnostic(Pose2d pose) {
    return withDriveToPose(() -> AllianceFlip.apply(pose));
  }

  public AutoBuilder withChoreoTraj(String name) {
    graph.add(new Node.ChoreoTraj(loadTraj(flippedTraj(name))));
    return this;
  }

  private static AutoBuilder swipeTemplate(
      String trajName, boolean endsIntakeToNeutral, boolean shouldFlipYAxis) {
    return new AutoBuilder(shouldFlipYAxis)
        .withChoreoTraj(trajName)
        .withDriveToPoseAllianceAgnostic(
            new Pose2d(3.4, 0.665, endsIntakeToNeutral ? Rotation2d.kZero : Rotation2d.k180deg));
  }

  public static Command doubleSwipe(
      Drive drive, SuperStructure superStructure, boolean shouldFlipYAxis) {
    return swipeTemplate("FullFuelSwipe", true, shouldFlipYAxis)
        .withDelay(4.2)
        .join(swipeTemplate("CleanSwipe", true, shouldFlipYAxis))
        .parallelAllWith(
            Commands.run(
                () ->
                    superStructure.setState(
                        FieldConstants.inAllianceZone(RobotState.getInstance().getEstimatedPose())
                            ? SuperStructureState.SHOOT
                            : SuperStructureState.INTAKE)))
        .withFinalState(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static Command experimentalSwipe(
      Drive drive, SuperStructure superStructure, boolean flipY) {
    return swipeTemplate("MogSwipe", true, flipY)
        .withDelay(4.2)
        .join(swipeTemplate("CleanSwipe", true, flipY))
        .parallelAllWith(
            Commands.run(
                () ->
                    superStructure.setState(
                        FieldConstants.inAllianceZone(RobotState.getInstance().getEstimatedPose())
                            ? SuperStructureState.SHOOT
                            : SuperStructureState.INTAKE)))
        .withFinalState(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static Command OP(Drive drive, SuperStructure superStructure, boolean flipY) {
    return new AutoBuilder(flipY)
        .withChoreoTraj("OPStart")
        .withChoreoTraj("OPEnd2")
        .withChoreoTraj("OPEscape")
        .parallelAllWith(
            Commands.run(
                () ->
                    superStructure.setState(
                        FieldConstants.inAllianceZone(RobotState.getInstance().getEstimatedPose())
                            ? SuperStructureState.SHOOT
                            : SuperStructureState.INTAKE)))
        .generate(drive, superStructure);
  }

  public static Command centerDepot(Drive drive, SuperStructure superStructure) {
    return new AutoBuilder(false)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CenterDepotCollect")
        .withFinalState(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static void registerAutoChoices(Drive drive, SuperStructure superStructure) {
    // autoChooser.addOption("2Swipe|R", doubleSwipe(drive, superStructure, false));
    // autoChooser.addOption("2Swipe|L", doubleSwipe(drive, superStructure, true));
    autoChooser.addOption("Mogged|R", experimentalSwipe(drive, superStructure, false));
    autoChooser.addOption("Mogged|L", experimentalSwipe(drive, superStructure, true));
    autoChooser.addOption("OP|R", OP(drive, superStructure, false));
    autoChooser.addOption("OP|L", OP(drive, superStructure, true));
    autoChooser.addOption("Depot|C", centerDepot(drive, superStructure));
  }
}
