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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
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
  private boolean autoshootNeutral;

  public AutoBuilder(boolean shouldFlipYAxis, boolean autoshootNeutral) {
    this.shouldFlipYAxis = shouldFlipYAxis;
    this.autoshootNeutral = autoshootNeutral;
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
    return Commands.sequence(commands.toArray(Command[]::new))
        .alongWith(
            Commands.defer(
                () ->
                    autoshootNeutral
                        ? Commands.run(
                            () -> {
                              if (FieldConstants.inAllianceZone(
                                  RobotState.getInstance().getEstimatedPose())) {
                                superStructure.setState(SuperStructureState.SHOOT);
                                superStructure.setTarget(TurretTarget.DEFAULT);
                              } else {
                                superStructure.setState(SuperStructureState.INTAKE);
                                superStructure.setTarget(TurretTarget.HUB);
                              }
                            })
                        : Commands.none(),
                autoshootNeutral ? Set.of(superStructure) : Set.of()));
  }

  private AutoBuilder append(AutoBuilder other) {
    graph.addAll(other.graph);
    return this;
  }

  public AutoBuilder join(AutoBuilder other) {
    return this.copy().append(other);
  }

  public AutoBuilder copy() {
    return new AutoBuilder(shouldFlipYAxis, autoshootNeutral).append(this);
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
    return new AutoBuilder(shouldFlipYAxis, true)
        .withChoreoTraj(trajName)
        .withDriveToPoseAllianceAgnostic(
            new Pose2d(3.4, 0.665, endsIntakeToNeutral ? Rotation2d.kZero : Rotation2d.k180deg));
  }

  public static Command doubleSwipe(
      Drive drive, SuperStructure superStructure, boolean shouldFlipYAxis) {
    return swipeTemplate("FullFuelSwipe", true, shouldFlipYAxis)
        .withDelay(4.2)
        .join(swipeTemplate("CleanSwipe", true, shouldFlipYAxis))
        .generate(drive, superStructure);
  }

  public static Command experimentalSwipe(
      Drive drive, SuperStructure superStructure, boolean flipY) {
    return swipeTemplate("MogSwipe", true, flipY)
        .withDelay(4.2)
        .join(swipeTemplate("CleanSwipe", true, flipY))
        .generate(drive, superStructure);
  }

  public static Command OP(Drive drive, SuperStructure superStructure, boolean flipY) {
    return new AutoBuilder(flipY, true)
        .withChoreoTraj("OPStart")
        .withChoreoTraj("OPEnd")
        .withChoreoTraj("OPEscape")
        .generate(drive, superStructure);
    // return new AutoBuilder(flipY, true).withChoreoTraj("OP").generate(drive, superStructure);
  }

  public static Command Orbit(Drive drive, SuperStructure superStructure, boolean flipY) {
    return new AutoBuilder(flipY, false)
        .withStateChange(SuperStructureState.SHOOT_INTAKE)
        .withChoreoTraj("OrbitPass")
        .generate(drive, superStructure);
  }

  /*
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

  private static void registerAuto(String name, Command auto) {
    autoChooser.addOption(name, auto);
  }

  public static void registerAutoChoices(Drive drive, SuperStructure superStructure) {
    registerAuto("2Swipe|R", doubleSwipe(drive, superStructure, false));
    registerAuto("2Swipe|L", doubleSwipe(drive, superStructure, true));
    registerAuto("Mogged|R", experimentalSwipe(drive, superStructure, false));
    registerAuto("Mogged|L", experimentalSwipe(drive, superStructure, true));
    registerAuto("OP|R", OP(drive, superStructure, false));
    registerAuto("OP|L", OP(drive, superStructure, true));
    registerAuto("Orbit|R", Orbit(drive, superStructure, false));
    registerAuto("Orbit|L", Orbit(drive, superStructure, true));
    /*
      registerAuto("Outpost|C", centerOutpost(drive, superStructure));
      registerAuto("Depot|C", centerDepot(drive, superStructure));
    */
  }
}
