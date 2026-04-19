package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.*;
import frc.robot.RobotConfig.DriveConstants.DriveState;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.StateSubsystem;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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

  private boolean SOTM = false;

  private SwerveSample choreoSample;
  private boolean firstSample = true;

  private Pose2d targetDrivePose = null;
  private SwerveRequest.ApplyRobotSpeeds robotRelativeRequest =
      new SwerveRequest.ApplyRobotSpeeds();
  private SwerveRequest.FieldCentric choreoRequest =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.Position);
  private SwerveRequest.FieldCentric fieldRequest =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withDeadband(0.1 * DriveConstants.maxDriveSpeedMps)
          .withRotationalDeadband(0.1 * DriveConstants.maxRotationSpeedRadPerSec);
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  private LoggedTunableNumber sotmSpeedFactor =
      new LoggedTunableNumber("Drive/sotmSpeedFactor", 0.2);
  private LoggedTunableNumber neutralSotmSpeedFactor =
      new LoggedTunableNumber("Drive/neutralSotmSpeedFactor", 0.45);

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

  public void setSOTMEnabled(boolean enabled) {
    SOTM = enabled;
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

  public void followChoreoTrajectory(SwerveSample sample) {
    this.choreoSample = sample;
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
        applyRequest(getInputRequest());
        break;
      case CHOREO:
        if (choreoSample != null) {
          if (firstSample) {
            firstSample = false;
            RobotState.getInstance().resetOdometry(choreoSample.getPose());
          }
          applyRequest(choreoSampleRequest(robotPose, choreoSample));
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
    return choreoRequest
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

    return robotRelativeRequest.withSpeeds(speeds);
  }

  private SwerveRequest getInputRequest() {
    double sx = -driveController.getLeftY() + 1e-8;
    double sy = -driveController.getLeftX() + 1e-8;
    double omega = -driveController.getRightX();

    omega = Math.copySign(omega * omega, omega);
    omega *= DriveConstants.maxRotationSpeedRadPerSec;

    double heading = Math.atan2(sy, sx);
    var magnitude = Math.hypot(sx, sy);
    magnitude = magnitude * magnitude; // heuristic
    magnitude *= DriveConstants.maxDriveSpeedMps;

    if (SOTM) {
      boolean neutral = !FieldConstants.inAllianceZone(inputs.Pose);
      magnitude *= neutral ? neutralSotmSpeedFactor.getAsDouble() : sotmSpeedFactor.getAsDouble();
      omega *= DriveConstants.SOTMOmegaFactor;
    }

    double xVelocity = magnitude * Math.cos(heading) * (FieldConstants.isBlueAlliance() ? 1 : -1);
    double yVelocity = magnitude * Math.sin(heading) * (FieldConstants.isBlueAlliance() ? 1 : -1);
    return fieldRequest.withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(omega);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return inputs.Speeds;
  }
}
