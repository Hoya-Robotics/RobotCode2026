package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotConfig;
import frc.robot.RobotState;
import frc.robot.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum DriveState {
  IDLE,
  TO_POSE,
  TELEOP
}

public class Drive extends StateSubsystem<DriveState> {
  private final GyroIO gyro;
  private GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();

  private final Module[] modules = new Module[4];
  private final XboxController driveController;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(RobotConfig.moduleTranslations);

  private PIDController linearController =
      new PIDController(
          RobotConfig.toPoseLinearGains.kp(),
          RobotConfig.toPoseLinearGains.ki(),
          RobotConfig.toPoseLinearGains.kd());
  private PIDController omegaController =
      new PIDController(
          RobotConfig.toPoseOmegaGains.kp(),
          RobotConfig.toPoseOmegaGains.ki(),
          RobotConfig.toPoseOmegaGains.kd());

  private Pose2d targetDrivePose = null;

  public Drive(XboxController controller, GyroIO gyro, ModuleIO[] moduleIOs) {
    this.driveController = controller;
    this.gyro = gyro;
    for (int i = 0; i < 4; ++i) {
      this.modules[i] = new Module(i, moduleIOs[i]);
    }

    linearController.setTolerance(RobotConfig.toPoseLinearGains.tolerance());
    omegaController.setTolerance(RobotConfig.toPoseOmegaGains.tolerance());
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroData);
    Logger.processInputs("Gyro", gyroData);

    for (var m : modules) m.periodic();

    statePeriodic();

    for (var m : modules) m.applyOutputs();
  }

  public void setTargetPose(Pose2d target) {
    this.targetDrivePose = target;
  }

  public void runSetpoint(ChassisSpeeds speeds) {
    var moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConfig.maxDriveSpeedMps);

    for (int i = 0; i < 4; ++i) {
      modules[i].runSetpoint(moduleStates[i]);
    }
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
      case TELEOP:
        runSetpoint(getControllerSpeeds());
        break;
      case TO_POSE:
        runSetpoint(getPidToPoseSetpoint());
        break;
      default:
        break;
    }
  }

  private ChassisSpeeds getPidToPoseSetpoint() {
    var robotPose = RobotState.getInstance().getEstimatedPose();
    var robotToTarget = targetDrivePose.minus(robotPose);
    double distance = robotToTarget.getTranslation().getNorm();
    Rotation2d heading = robotToTarget.getRotation();

    double linearOutput =
        Math.min(Math.abs(linearController.calculate(distance, 0.0)), RobotConfig.maxDriveSpeedMps);
    double vx = linearOutput * heading.getCos();
    double vy = linearOutput * heading.getSin();

    double omega =
        MathUtil.clamp(
            omegaController.calculate(
                robotPose.getRotation().getRadians(), targetDrivePose.getRotation().getRadians()),
            -RobotConfig.maxRotationSpeedRps,
            RobotConfig.maxRotationSpeedRps);

    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyroData.yaw);
  }

  private ChassisSpeeds getControllerSpeeds() {
    double sx = -driveController.getLeftY();
    double sy = -driveController.getLeftX();
    double omega = -driveController.getRightX();

    omega = MathUtil.applyDeadband(omega, RobotConfig.controllerDeadband);
    omega = omega * omega * Math.signum(omega); // heuristic

    var heading = new Rotation2d(sx, sy);
    var magnitude = MathUtil.applyDeadband(Math.hypot(sx, sy), RobotConfig.controllerDeadband);
    magnitude = magnitude * magnitude; // heuristic

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        magnitude * heading.getCos(), magnitude * heading.getSin(), omega, gyroData.yaw);
  }
}
