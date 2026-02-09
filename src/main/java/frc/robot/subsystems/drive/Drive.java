package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.*;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.util.StateSubsystem;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

enum DriveState {
  IDLE,
  TO_POSE,
  TELEOP,
  AUTOPILOT
}

public class Drive extends StateSubsystem<DriveState> {
  private final DriveIO io;
  private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final XboxController driveController;

  private PIDController linearController = DriveConstants.toPoseLinearGains.toController();
  private PIDController omegaController = DriveConstants.toPoseOmegaGains.toController();

  private Pose2d targetDrivePose = null;
  private int apIndex = 0;
  private Optional<List<APTarget>> apTargets = Optional.empty();

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

  private static final APConstraints apConstraints =
      new APConstraints().withVelocity(4.0).withAcceleration(11.0).withJerk(2.0);

  private static final APProfile apProfile =
      new APProfile(apConstraints)
          .withErrorXY(Units.Centimeter.of(4))
          .withErrorTheta(Units.Degrees.of(1.5));
  private static final Autopilot autopilot = new Autopilot(apProfile);

  public Drive(XboxController controller, DriveIO io) {
    this.driveController = controller;
    this.io = io;

    linearController.setTolerance(DriveConstants.toPoseLinearTolerance);
    omegaController.setTolerance(DriveConstants.toPoseThetaTolerance);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    setState(DriveState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("DriveInputs", inputs);
    Logger.recordOutput("Drive/systemState", getCurrentState());

    if (DriverStation.isTeleop() && getCurrentState() == DriveState.IDLE) {
      setState(DriveState.TELEOP);
    }

    applyState();
  }

  public void autopilotTo(List<APTarget> targets) {
    this.apTargets = Optional.of(targets);
    apIndex = 0;
    setState(DriveState.AUTOPILOT);
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
      default:
        break;
    }

    return requestedState;
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case AUTOPILOT:
        if (apTargets.isPresent()) {
          applyRequest(autopilotRequest(inputs.Pose));
          // applyRequest(autopilotRequest(RobotState.getInstance().getSimulatedDrivePose()));
        } else {
          setState(DriveState.TELEOP);
        }
        break;
      case TELEOP:
        applyRequest(getControllerRequest());
        break;
      case TO_POSE:
        applyRequest(
            getPidToPoseRequest(
                RobotState.getInstance().getSimulatedDrivePose(), Optional.empty()));
        break;
      case IDLE:
        applyRequest(brakeRequest);
        break;
    }
  }

  private SwerveRequest autopilotRequest(Pose2d robotPose) {
    var target = apTargets.get().get(apIndex);

    if (autopilot.atTarget(robotPose, target)) {
      apIndex += 1;
      if (apIndex == apTargets.get().size()) {
        setState(DriveState.IDLE);
        return brakeRequest;
      }
      target = apTargets.get().get(apIndex);
    }

    APResult output = autopilot.calculate(robotPose, inputs.Speeds, target);

    Logger.recordOutput("Drive/autopilot/targetPose", target.getReference());
    Logger.recordOutput("Drive/autopilot/vx", output.vx());
    Logger.recordOutput("Drive/autopilot/vy", output.vy());
    Logger.recordOutput("Drive/autopilot/heading", output.targetAngle());

    return driveAtAngle
        .withVelocityX(output.vx())
        .withVelocityY(output.vy())
        .withTargetDirection(output.targetAngle());
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
