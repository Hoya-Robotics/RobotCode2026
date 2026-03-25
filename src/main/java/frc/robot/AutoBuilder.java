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
  sealed interface Node
      permits Node.Wait, Node.DriveToPose, Node.StateChange, Node.ChoreoTraj, Node.Cmd {
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

    record StateChange(SuperStructureState state) implements Node {
      public Command toCommand(Drive drive, SuperStructure superStructure, Timer timer) {
        return superStructure.setStateCommand(state);
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

    for (int i = 0; i < graph.size(); ++i) {
      Node node = graph.get(i);
      commands.add(node.toCommand(drive, superStructure, autoTimer));
      String key = "Auto/" + i + "-" + node.getClass().getSimpleName();
      commands.add(Commands.runOnce(() -> Logger.recordOutput(key, autoTimer.get())));
    }
    return Commands.sequence(commands.toArray(Command[]::new));
  }

  private AutoBuilder append(AutoBuilder other) {
    graph.addAll(other.graph);
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
    graph.add(new Node.StateChange(state));
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
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj(trajName)
        .withDriveToPoseAllianceAgnostic(
            new Pose2d(3.4, 0.665, endsIntakeToNeutral ? Rotation2d.kZero : Rotation2d.k180deg));
  }

  private static AutoBuilder cleanSwipeTemplate(boolean shouldFlipYAxis) {
    return swipeTemplate("CleanSwipe", true, shouldFlipYAxis);
  }

  private static AutoBuilder fullSwipeTemplate(boolean shouldFlipYAxis) {
    return swipeTemplate("FullFuelSwipe", true, shouldFlipYAxis);
  }

  public static Command doubleSwipe(
      Drive drive, SuperStructure superStructure, boolean shouldFlipYAxis) {
    return fullSwipeTemplate(shouldFlipYAxis)
        .withDelay(4.2)
        .join(cleanSwipeTemplate(shouldFlipYAxis))
        .generate(drive, superStructure);
  }

  public static Command experimentalSwipe(
      Drive drive, SuperStructure superStructure, boolean flipY) {
    return swipeTemplate("MogSwipe", true, flipY)
        .withDelay(4.2)
        .join(cleanSwipeTemplate(flipY))
        .generate(drive, superStructure);
  }

  /*
  public static Command doubleSwipeOutpost(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate(false)
        .withDelay(2.75)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CleanSwipe")
        .withDriveToPose(
            () -> AllianceFlip.apply(new Pose2d(FieldConstants.humanOutpost, Rotation2d.kZero)))
        .withDelayTillRemaining(1.25)
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("LeaveOutpost")
        .generate(drive, superStructure);
  }

  public static Command swipeOutpost(Drive drive, SuperStructure superStructure) {
    return fullSwipeTemplate(false)
        .withDelay(2.75)
        .withDriveToPose(
            () -> AllianceFlip.apply(new Pose2d(FieldConstants.humanOutpost, Rotation2d.kZero)))
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
    return new AutoBuilder(true)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("CenterDepot")
        .withStateChange(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }

  public static Command centerOutpost(Drive drive, SuperStructure superStructure) {
    return new AutoBuilder(true)
        .withStateChange(SuperStructureState.IDLE)
        .withChoreoTraj("CenterOutpost")
        .withStateChange(SuperStructureState.SHOOT)
        .generate(drive, superStructure);
  }*/

  public static Command passSweep(Drive drive, SuperStructure superStructure, boolean flipY) {
    return new AutoBuilder(flipY)
        .withStateChange(SuperStructureState.SHOOT_INTAKE)
        .withChoreoTraj("OrbitPass")
        .generate(drive, superStructure);
  }

  public static Command OP(Drive drive, SuperStructure superStructure) {
    return new AutoBuilder(false)
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("OPStart")
        .withStateChange(SuperStructureState.INTAKE)
        .withChoreoTraj("OPSecond")
        .generate(drive, superStructure);
  }

  private static void registerAuto(String name, Command auto) {
    autoChooser.addOption(name, auto);
  }

  public static void registerAutoChoices(Drive drive, SuperStructure superStructure) {
    registerAuto("2Swipe|R", doubleSwipe(drive, superStructure, false));
    registerAuto("2Swipe|L", doubleSwipe(drive, superStructure, true));
    registerAuto("Mogged|R", experimentalSwipe(drive, superStructure, false));
    registerAuto("Mogged|L", experimentalSwipe(drive, superStructure, true));

    registerAuto("OP", OP(drive, superStructure));
    registerAuto("Orbit", passSweep(drive, superStructure, false));

    /*
      registerAuto("0xOutpost", centerOutpost(drive, superStructure));
      registerAuto("1xOutpost", swipeOutpost(drive, superStructure));
      registerAuto("2xOutpost", doubleSwipeOutpost(drive, superStructure));

      registerAuto("0xDepot", centerDepot(drive, superStructure));
      registerAuto("1xDepot", swipeAndDepot(drive, superStructure));
      registerAuto("2xDepot", doubleSwipeDepot(drive, superStructure));
    */
  }
}
