package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.*;
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
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.util.StateSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

enum DriveState {
  IDLE,
  TO_POSE,
  TELEOP,
  CHOREO,
  TRENCH,
}

public class Drive extends StateSubsystem<DriveState> {
  private final DriveIO io;
  private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final XboxController driveController;

  private final PIDController linearController = DriveConstants.toPoseLinearGains.toController();
  private final PIDController omegaController = DriveConstants.toPoseOmegaGains.toController();
  private final PIDController choreoXController = DriveConstants.choreoLinearGains.toController();
  private final PIDController choreoYController = DriveConstants.choreoLinearGains.toController();
  private final PIDController choreoThetaController =
      DriveConstants.choreoThetaGains.toController();
  private final PIDController trenchYController = DriveConstants.trenchYGains.toController();

  private Pose2d targetDrivePose = null;
  private Optional<Trajectory<SwerveSample>> choreoTrajectory = Optional.empty();
  private Timer choreoTimer = new Timer();

  private SwerveRequest.ApplyRobotSpeeds robotRelativeRequest =
      new SwerveRequest.ApplyRobotSpeeds();
  private SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withHeadingPID(4, 0, 0);
  private SwerveRequest.FieldCentric fieldRequest =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position)
          .withDeadband(0.1 * DriveConstants.maxDriveSpeedMps)
          .withRotationalDeadband(0.1 * DriveConstants.maxRotationSpeedRadPerSec);
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  public Drive(XboxController controller, DriveIO io) {
    this.driveController = controller;
    this.io = io;

    linearController.setTolerance(DriveConstants.toPoseLinearTolerance);
    omegaController.setTolerance(DriveConstants.toPoseThetaTolerance);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

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

  public void followChoreoTrajectory(Trajectory<SwerveSample> traj) {
    choreoTrajectory = Optional.of(traj);
    setState(DriveState.CHOREO);
  }

  public void driveToPose(Pose2d target) {
    this.targetDrivePose = target;
    setState(DriveState.TO_POSE);
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
        choreoYController.reset();
        choreoXController.reset();
        choreoThetaController.reset();
        ;
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
        if (shouldAlignTrench(robotPose)) setState(DriveState.TRENCH);
        break;
      case CHOREO:
        if (choreoTrajectory.isPresent()) applyRequest(choreoRequest(robotPose));
        break;
      case TRENCH:
        Pose2d trenchPose = robotPose.nearest(FieldConstants.trenchPoses);
        applyRequest(trenchRequest(robotPose, trenchPose));
        if (robotPose.getTranslation().getDistance(trenchPose.getTranslation())
            < Units.inchesToMeters(2.0)) setState(DriveState.TELEOP);
        break;
      case TO_POSE:
        applyRequest(getPidToPoseRequest(robotPose, Optional.empty()));
        break;
      case IDLE:
        applyRequest(brakeRequest);
        break;
    }
  }

  // TODO: how do we decide when to align?
  // - Maybe align sooner if we are pointing in trench direction
  // / dont align if we arent driving very fast
  private boolean shouldAlignTrench(Pose2d robotPose) {
    Pose2d trenchPose = robotPose.nearest(FieldConstants.trenchPoses);
    return robotPose.getTranslation().getDistance(trenchPose.getTranslation())
        < Units.inchesToMeters(30);
  }

  // Match requested x joystick with output y to align to center of trench
  private SwerveRequest trenchRequest(Pose2d robotPose, Pose2d trenchPose) {
    double xOutput = getInputVector().get(0);
    var robotToTrench = trenchPose.minus(robotPose);

    double yOutput = trenchYController.calculate(robotToTrench.getY());
    double omega =
        omegaController.calculate(
            robotPose.getRotation().getRadians(), trenchPose.getRotation().getRadians());

    return fieldRequest.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(omega);
  }

  private SwerveRequest choreoRequest(Pose2d robotPose) {
    if (!choreoTimer.isRunning()) choreoTimer.restart();
    var sampleAt = choreoTrajectory.get().sampleAt(choreoTimer.get(), false);
    if (sampleAt.isEmpty() || choreoTimer.get() > choreoTrajectory.get().getTotalTime()) {
      choreoTrajectory = Optional.empty();
      setState(DriveState.TELEOP);
      choreoTimer.stop();
      return brakeRequest;
    }

    var sample = sampleAt.get();

    Logger.recordOutput("Drive/choreo/elapsedTime", choreoTimer.get());
    Logger.recordOutput("Drive/choreo/totalTime", choreoTrajectory.get().getTotalTime());
    Logger.recordOutput("Drive/choreo/trajectoryName", choreoTrajectory.get().name());
    Logger.recordOutput("Drive/choreo/target", sample.getPose());

    double vx = sample.vx + choreoXController.calculate(robotPose.getX(), sample.x);
    double vy = sample.vy + choreoYController.calculate(robotPose.getY(), sample.y);
    double omega =
        sample.omega
            + choreoThetaController.calculate(robotPose.getRotation().getRadians(), sample.heading);
    var requestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, inputs.gyroYaw);

    Logger.recordOutput("Drive/choreo/vx", requestedSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Drive/choreo/vy", requestedSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Drive/choreo/omega", requestedSpeeds.omegaRadiansPerSecond);

    return fieldRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
  }

  private SwerveRequest getPidToPoseRequest(Pose2d robotPose, Optional<Pose2d> targetPose) {
    var target = targetPose.orElse(targetDrivePose);
    var robotToTarget = target.getTranslation().minus(robotPose.getTranslation());
    double distance = robotToTarget.getNorm();
    Rotation2d heading = robotToTarget.getAngle();

    double linearOutput =
        Math.min(
            Math.abs(linearController.calculate(distance, 0.0)), DriveConstants.maxDriveSpeedMps);
    double vx = linearOutput * heading.getCos();
    double vy = linearOutput * heading.getSin();

    double omega =
        MathUtil.clamp(
            omegaController.calculate(
                robotPose.getRotation().getRadians(), target.getRotation().getRadians()),
            -DriveConstants.maxRotationSpeedRadPerSec,
            DriveConstants.maxRotationSpeedRadPerSec);

    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, inputs.gyroYaw);

    Logger.recordOutput("Drive/ToPose/TargetPose", target);
    Logger.recordOutput("Drive/ToPose/linearError", distance);
    Logger.recordOutput("Drive/ToPose/headingDeg", heading.getDegrees());
    Logger.recordOutput("Drive/ToPose/linearOutput", linearOutput);
    Logger.recordOutput("Drive/ToPose/OmegaOutput", omega);
    Logger.recordOutput("Drive/ToPose/speeds", speeds);

    return robotRelativeRequest.withSpeeds(speeds);
  }

  private Vector<N3> getInputVector() {
    double sx = -driveController.getLeftY();
    double sy = -driveController.getLeftX();
    double omega = -driveController.getRightX();

    omega = Math.copySign(omega * omega, omega);
    omega *= DriveConstants.maxRotationSpeedRadPerSec;

    var heading = new Rotation2d(sx, sy);
    var magnitude = Math.hypot(sx, sy);
    magnitude = magnitude * magnitude; // heuristic
    magnitude *= DriveConstants.maxDriveSpeedMps;

    return VecBuilder.fill(magnitude * heading.getCos(), magnitude * heading.getSin(), omega);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return inputs.Speeds;
  }
}
