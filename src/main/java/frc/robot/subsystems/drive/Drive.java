package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.util.StateSubsystem;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

enum DriveState {
  IDLE,
  TO_POSE,
  TELEOP,
  CHOREO
}

public class Drive extends StateSubsystem<DriveState> {
  private final DriveIO io;
  private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final CommandXboxController driveController;

  private final PIDController linearController = DriveConstants.toPoseLinearGains.toController();
  private final PIDController omegaController = DriveConstants.toPoseOmegaGains.toController();

  private final PIDController choreoXController = DriveConstants.choreoXGains.toController();
  private final PIDController choreoYController = DriveConstants.choreoYGains.toController();
  private final PIDController choreoOmegaController =
      DriveConstants.choreoOmegaGains.toController();

  private Timer choreoTimer = new Timer();
  private Optional<Trajectory<SwerveSample>> choreoTrajectory = Optional.empty();

  private double teleopSpeedLimit = DriveConstants.maxDriveSpeedMps;
  private Pose2d trenchPose = null;
  private Pose2d targetDrivePose = null;
  private SwerveRequest.ApplyRobotSpeeds robotRelativeRequest =
      new SwerveRequest.ApplyRobotSpeeds();
  private SwerveRequest.FieldCentric fieldRequest =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withDeadband(0.1 * DriveConstants.maxDriveSpeedMps)
          .withRotationalDeadband(0.1 * DriveConstants.maxRotationSpeedRadPerSec);
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  public Drive(CommandXboxController controller, DriveIO io) {
    this.driveController = controller;
    this.io = io;

    linearController.setTolerance(DriveConstants.toPoseLinearTolerance);
    omegaController.setTolerance(DriveConstants.toPoseThetaTolerance);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    choreoOmegaController.enableContinuousInput(-Math.PI, Math.PI);

    setState(DriveState.IDLE);
    RobotState.getInstance().registerDrivetrain(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("DriveInputs", inputs);
    Logger.recordOutput("Drive/systemState", getCurrentState());

    RobotState.getInstance().addDriveInputs(inputs);

    if (DriverStation.isTeleop() && getCurrentState() == DriveState.IDLE) {
      setState(DriveState.TELEOP);
    }

    applyState();
  }

  public void setIdle() {
    setState(DriveState.IDLE);
  }

  public void setTeleopSpeedLimit(double limit) {
    teleopSpeedLimit = limit;
  }

  public Command driveToPoseCommand(Pose2d target) {
    return Commands.runOnce(() -> this.driveToPose(target))
        .andThen(Commands.waitUntil(() -> atDriveToPoseSetpoint()))
        .andThen(Commands.runOnce(this::setIdle));
  }

  public Command driveToPoseCommandDeferred(Supplier<Pose2d> target) {
    return Commands.defer(
        () ->
            Commands.runOnce(() -> this.driveToPose(target.get()))
                .andThen(Commands.waitUntil(() -> atDriveToPoseSetpoint()))
                .andThen(Commands.runOnce(this::setIdle)),
        Set.of(this));
  }

  public void driveToPose(Pose2d target) {
    this.targetDrivePose = target;

    setState(DriveState.TO_POSE);
  }

  public Command followChoreoTrajectoryCommand(Trajectory<SwerveSample> trajectory) {
    return Commands.runOnce(() -> this.followChoreoTrajectory(trajectory))
        .andThen(Commands.waitUntil(() -> choreoTrajectoryDone()));
  }

  public void followChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
    this.choreoTrajectory = Optional.of(trajectory);
    setState(DriveState.CHOREO);
  }

  public void addVisionMeasurement(VisionObservation observation) {
    io.addVisionMeasurement(observation.pose(), observation.timestamp(), observation.stdDevs());
  }

  public void resetOdometry(Pose2d override) {
    io.resetOdometry(override);
  }

  public void applyRequest(SwerveRequest request) {
    io.applyRequest(request);
  }

  @Override
  public DriveState handleStateTransitions() {
    switch (requestedState) {
      case TO_POSE:
        linearController.reset();
        omegaController.reset();
        break;
      case CHOREO:
        choreoTimer.restart();
        break;
      default:
        break;
    }

    return requestedState;
  }

  @Override
  public void applyState() {
    Pose2d robotPose = inputs.Pose;
    switch (getCurrentState()) {
      case TELEOP:
        var speeds = getInputVector();
        applyRequest(
            fieldRequest
                .withVelocityX(speeds.get(0))
                .withVelocityY(speeds.get(1))
                .withRotationalRate(speeds.get(2)));
        break;
      case CHOREO:
        if (choreoTrajectory.isPresent()) {
          Logger.recordOutput("Drive/Choreo/trajectoryName", choreoTrajectory.get().name());
          Logger.recordOutput("Drive/Choreo/timer", choreoTimer.get());
          Logger.recordOutput("Drive/Choreo/trajectoryTime", choreoTrajectory.get().getTotalTime());
          Optional<SwerveSample> sample =
              choreoTrajectory.get().sampleAt(choreoTimer.get(), !FieldConstants.isBlueAlliance());
          if (sample.isPresent()) {
            applyRequest(choreoSampleRequest(robotPose, sample.get()));
          }
        }
        break;
      case TO_POSE:
        applyRequest(getPidToPoseRequest(robotPose, Optional.empty()));
        break;
      case IDLE:
        applyRequest(brakeRequest);
        break;
    }
  }

  public boolean choreoTrajectoryDone() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose2d finalPose =
        choreoTrajectory.get().getFinalSample(!FieldConstants.isBlueAlliance()).get().getPose();
    double linearErr = finalPose.getTranslation().getDistance(robotPose.getTranslation());
    double headingErr = robotPose.getRotation().minus(finalPose.getRotation()).getDegrees();
    Logger.recordOutput("Drive/Choreo/linearError", linearErr);
    Logger.recordOutput("Drive/Choreo/headingErr", headingErr);
    return choreoTimer.get() - choreoTrajectory.get().getTotalTime() > 0.0;
  }

  public boolean atDriveToPoseSetpoint() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    double dist = robotPose.getTranslation().getDistance(targetDrivePose.getTranslation());
    double angleError = robotPose.getRotation().minus(targetDrivePose.getRotation()).getDegrees();
    return dist < Units.inchesToMeters(3.0)
        && angleError < 2.0
        && Math.hypot(inputs.Speeds.vxMetersPerSecond, inputs.Speeds.vyMetersPerSecond)
            < DriveConstants.toPoseEndSpeed;
  }

  private SwerveRequest choreoSampleRequest(Pose2d pose, SwerveSample sample) {
    Logger.recordOutput("Drive/Choreo/targetPose", sample.getPose());
    return fieldRequest
        .withVelocityX(sample.vx + choreoXController.calculate(pose.getX(), sample.x))
        .withVelocityY(sample.vy + choreoYController.calculate(pose.getY(), sample.y))
        .withRotationalRate(
            sample.omega
                + choreoOmegaController.calculate(pose.getRotation().getRadians(), sample.heading));
  }

  private SwerveRequest getPidToPoseRequest(Pose2d robotPose, Optional<Pose2d> targetPose) {
    var target = targetPose.orElse(targetDrivePose);
    var robotToTarget = target.getTranslation().minus(robotPose.getTranslation());
    double distance = robotToTarget.getNorm();
    Rotation2d headingToTarget = robotToTarget.getAngle();

    double linearOutput =
        Math.min(
            Math.abs(linearController.calculate(distance, 0.0)), DriveConstants.maxDriveSpeedMps);
    double vx = linearOutput * headingToTarget.getCos();
    double vy = linearOutput * headingToTarget.getSin();
    Rotation2d targetHeading = target.getRotation();

    double omega =
        MathUtil.clamp(
            omegaController.calculate(
                robotPose.getRotation().getRadians(), targetHeading.getRadians()),
            -DriveConstants.maxRotationSpeedRadPerSec,
            DriveConstants.maxRotationSpeedRadPerSec);

    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, inputs.gyroYaw);
    Logger.recordOutput("Drive/ToPose/TargetPose", target);
    Logger.recordOutput("Drive/ToPose/linearError", distance);
    Logger.recordOutput("Drive/ToPose/vx", vx);
    Logger.recordOutput("Drive/ToPose/vy", vy);
    Logger.recordOutput("Drive/ToPose/omega", omega);
    return robotRelativeRequest.withSpeeds(speeds);
  }

  private Vector<N3> getInputVector() {
    double sx = -driveController.getLeftY() + 1e-8;
    double sy = -driveController.getLeftX() + 1e-8;
    double omega = -driveController.getRightX();

    omega = Math.copySign(omega * omega, omega);
    omega *= DriveConstants.maxRotationSpeedRadPerSec;

    var heading = new Rotation2d(sx, sy);
    var magnitude = Math.hypot(sx, sy);
    magnitude = magnitude * magnitude; // heuristic
    magnitude *= DriveConstants.maxDriveSpeedMps;

    if (magnitude > teleopSpeedLimit) magnitude = teleopSpeedLimit;
    double xVelocity = magnitude * heading.getCos() * (FieldConstants.isBlueAlliance() ? 1 : -1);
    double yVelocity = magnitude * heading.getSin() * (FieldConstants.isBlueAlliance() ? 1 : -1);
    return VecBuilder.fill(xVelocity, yVelocity, omega);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return inputs.Speeds;
  }
}
