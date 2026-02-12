package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.util.StateSubsystem;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

enum DriveState {
  IDLE,
  TO_POSE,
  TELEOP,
  CHOREO,
  AUTOPILOT
}

public class Drive extends StateSubsystem<DriveState> {
  private final DriveIO io;
  private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final XboxController driveController;
  private static final Autopilot autopilot = new Autopilot(RobotConfig.DriveConstants.apProfile);

  private PIDController linearController = DriveConstants.toPoseLinearGains.toController();
  private PIDController omegaController = DriveConstants.toPoseOmegaGains.toController();
  private PIDController choreoXController = DriveConstants.choreoLinearGains.toController();
  private PIDController choreoYController = DriveConstants.choreoLinearGains.toController();
  private PIDController choreoThetaController = DriveConstants.choreoThetaGains.toController();

  private Pose2d targetDrivePose = null;
  private Optional<Trajectory<SwerveSample>> choreoTrajectory = Optional.empty();
  private Queue<APTarget> autopilotPath;
  private Timer choreoTimer = new Timer();

  private SwerveRequest.ApplyRobotSpeeds robotRelativeRequest =
      new SwerveRequest.ApplyRobotSpeeds();
  private SwerveRequest.FieldCentricFacingAngle driveAtAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withHeadingPID(4, 0, 0);
  private SwerveRequest.FieldCentric fieldRequest =
      new SwerveRequest.FieldCentric().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
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

  public void traverseTrench() {
    var poses = FieldConstants.nearestTrenchWaypoints(inputs.Pose);
    APTarget entrance =
        new APTarget(poses.getFirst())
            .withEntryAngle(poses.getFirst().getRotation())
            .withVelocity(3.0);
    APTarget exit = new APTarget(poses.getSecond()).withVelocity(4.0);

    autopilotPath = new LinkedList<>();
    autopilotPath.add(entrance);
    autopilotPath.add(exit);

    setState(DriveState.AUTOPILOT);
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
        applyRequest(getControllerRequest());
        break;
      case CHOREO:
        if (choreoTrajectory.isPresent()) {
          applyRequest(choreoRequest(robotPose));
        }
        break;
      case TO_POSE:
        applyRequest(getPidToPoseRequest(robotPose, Optional.empty()));
        break;
      case AUTOPILOT:
        var target = autopilotPath.peek();
        while (target != null && autopilot.atTarget(robotPose, target)) {
          autopilotPath.remove();
          target = autopilotPath.peek();
        }
        if (target == null) {
          setState(DriveState.IDLE);
        } else {
          applyRequest(autopilotRequest(target, robotPose));
        }
        break;
      case IDLE:
        applyRequest(brakeRequest);
        break;
    }
  }

  private SwerveRequest autopilotRequest(APTarget target, Pose2d robotPose) {
    var output = autopilot.calculate(robotPose, inputs.Speeds, target);

    return driveAtAngle
        .withVelocityX(output.vx())
        .withVelocityY(output.vy())
        .withTargetDirection(output.targetAngle());
  }

  private SwerveRequest choreoRequest(Pose2d robotPose) {
    if (!choreoTimer.isRunning()) {
      choreoTimer.restart();
    }
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
    Logger.recordOutput("Drive/choreo/vx", sample.vx);
    Logger.recordOutput("Drive/choreo/vy", sample.vy);
    Logger.recordOutput("Drive/choreo/omega", sample.omega);
    Logger.recordOutput("Drive/choreo/target", sample.getPose());

    return fieldRequest
        .withVelocityX(sample.vx + choreoXController.calculate(robotPose.getX(), sample.x))
        .withVelocityY(sample.vy + choreoYController.calculate(robotPose.getY(), sample.y))
        .withRotationalRate(
            sample.omega
                + choreoThetaController.calculate(
                    robotPose.getRotation().getRadians(), sample.heading));
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
            -DriveConstants.maxRotationSpeedRps,
            DriveConstants.maxRotationSpeedRps);

    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, inputs.gyroYaw);

    Logger.recordOutput("Drive/ToPose/TargetPose", target);
    Logger.recordOutput("Drive/ToPose/linearError", distance);
    Logger.recordOutput("Drive/ToPose/headingDeg", heading.getDegrees());
    Logger.recordOutput("Drive/ToPose/linearOutput", linearOutput);
    Logger.recordOutput("Drive/ToPose/OmegaOutput", omega);
    Logger.recordOutput("Drive/ToPose/speeds", speeds);

    return robotRelativeRequest.withSpeeds(speeds);
  }

  private SwerveRequest getControllerRequest() {
    double sx = -driveController.getLeftY();
    double sy = -driveController.getLeftX();
    double omega = -driveController.getRightX();

    omega = MathUtil.applyDeadband(omega, DriveConstants.controllerDeadband);
    omega = omega * omega * Math.signum(omega); // heuristic

    var heading = new Rotation2d(sx, sy);
    var magnitude = MathUtil.applyDeadband(Math.hypot(sx, sy), DriveConstants.controllerDeadband);
    magnitude = magnitude * magnitude; // heuristic

    return fieldRequest
        .withVelocityX(magnitude * heading.getCos())
        .withVelocityY(magnitude * heading.getSin())
        .withRotationalRate(omega);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return inputs.Speeds;
  }
}
