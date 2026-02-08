package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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
  CHOREO_PATH
}

public class Drive extends StateSubsystem<DriveState> {
  private final DriveIO io;
  private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final XboxController driveController;

  private PIDController linearController = DriveConstants.toPoseLinearGains.toController();
  private PIDController omegaController = DriveConstants.toPoseOmegaGains.toController();
  private PIDController choreoXController = DriveConstants.choreoXGains.toController();
  private PIDController choreoYController = DriveConstants.choreoYGains.toController();
  private PIDController choreoThetaController = DriveConstants.choreoThetaGains.toController();

  private Pose2d targetDrivePose = null;
  private Optional<Trajectory<SwerveSample>> choreoTrajectory = Optional.empty();
  private Timer choreoTimer = new Timer();

  private SwerveRequest.ApplyRobotSpeeds robotRelativeRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  public Drive(XboxController controller, DriveIO io) {
    this.driveController = controller;
    this.io = io;

    linearController.setTolerance(DriveConstants.toPoseLinearTolerance);
    omegaController.setTolerance(DriveConstants.toPoseThetaTolerance);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    setState(DriveState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    Logger.recordOutput("Drive/systemState", getCurrentState());

    if (DriverStation.isTeleop() && getCurrentState() == DriveState.IDLE) {
      setState(DriveState.TELEOP);
    }

    applyState();
  }

  public void followTrajectory(Trajectory<SwerveSample> traj) {
    choreoTrajectory = Optional.of(traj);
    setState(DriveState.CHOREO_PATH);
  }

  public void driveToPose(Pose2d target) {
    this.targetDrivePose = target;
    setState(DriveState.TO_POSE);
  }

  public void resetOdometry(Pose2d override) {
    io.resetOdometry(override);
    RobotState.getInstance().hardSetOdometry(override);
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
      case CHOREO_PATH:
        choreoTimer.stop();
        choreoXController.reset();
        choreoYController.reset();
        choreoThetaController.reset();
        break;
      default:
        break;
    }

    return requestedState;
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case TELEOP:
        // applyRequest(getControllerRequest());
        applyRequest(
            new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        new ChassisSpeeds(4.0, 0.0, 0.0), inputs.gyroYaw)));
        break;
      case TO_POSE:
        applyRequest(
            getPidToPoseRequest(
                RobotState.getInstance().getSimulatedDrivePose(), Optional.empty()));
        break;
        /*
        case CHOREO_PATH:
          if (choreoTrajectory.isPresent()) {
            if (!choreoTimer.isRunning()) choreoTimer.restart();
            Optional<SwerveSample> sampleAt =
                choreoTrajectory.get().sampleAt(choreoTimer.get(), false);
            Logger.recordOutput("Drive/Choreo/Time", choreoTimer.get());
            Logger.recordOutput("Drive/Choreo/Total Time", choreoTrajectory.get().getTotalTime());
            Logger.recordOutput("Drive/Choreo/Traj Name", choreoTrajectory.get().name());
            Logger.recordOutput("Drive/Choreo/isSample", sampleAt.isPresent());

            if (choreoTimer.get() > choreoTrajectory.get().getTotalTime()) {
              setState(DriveState.IDLE);
            }

            sampleAt.ifPresent(
                (sample) -> {
                  var pose = RobotState.getInstance().getSimulatedDrivePose();
                  Logger.recordOutput("Drive/Choreo/feedforwardVx", sample.vx);
                  Logger.recordOutput("Drive/Choreo/feedforwardVy", sample.vy);
                  var setpoint =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          sample.vx + choreoXController.calculate(pose.getX(), sample.x),
                          sample.vy + choreoYController.calculate(pose.getY(), sample.y),
                          sample.omega
                              + choreoThetaController.calculate(
                                  pose.getRotation().getRadians(), sample.heading),
                          gyroData.yaw);
            io.applyRequest(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(setpoint));
                });
          }
          break;*/
      case IDLE:
        applyRequest(new SwerveRequest.SwerveDriveBrake());
        break;
    }
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

    return new SwerveRequest.FieldCentric()
        .withVelocityX(magnitude * heading.getCos())
        .withVelocityY(magnitude * heading.getSin())
        .withRotationalRate(omega);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return inputs.Speeds;
  }
}
