package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateSubsystem<frc.robot.subsystems.drive.Drive.SystemState> {
  public enum SystemState {
    IDLE,
    TELEOP_DRIVE,
    TO_POSE_PID,
  };

  private final Module[] swerveModules = new Module[4];
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.modulePositions);

  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();

	// Drive to pose controllers
  private final PIDController linearController =
      new PIDController(
          DriveConstants.driveGains.kp(),
          DriveConstants.driveGains.ki(),
          DriveConstants.driveGains.kd());
  private final PIDController omegaController =
      new PIDController(
          DriveConstants.rotGains.kp(), DriveConstants.rotGains.ki(), DriveConstants.rotGains.kd());

  private final XboxController controller;
  private Pose2d driveToPointPose = new Pose2d();

  public Drive(
      GyroIO gyro,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      XboxController controller) {
    this.controller = controller;
    this.gyro = gyro;
    this.swerveModules[0] = new Module(flModuleIO, 0);
    this.swerveModules[1] = new Module(frModuleIO, 1);
    this.swerveModules[2] = new Module(blModuleIO, 2);
    this.swerveModules[3] = new Module(brModuleIO, 3);

    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    linearController.setTolerance(DriveConstants.driveTolerance.in(Meters));
    omegaController.setTolerance(DriveConstants.rotateTolerance.in(Radians));
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroData);
    Logger.processInputs("Drive/Gryo", gyroData);
    Logger.recordOutput("Drive/systemState", getCurrentState());

    for (var m : swerveModules) {
      m.periodic();
    }

    statePeriodic();

    for (var m : swerveModules) {
      m.periodicAfter();
    }
  }

  @Override
  protected void applyState() {
    switch (getCurrentState()) {
      case TELEOP_DRIVE:
        runVelocity(getControllerSpeeds());
        break;
      case TO_POSE_PID:
        pidToPose();
        break;
      default:
        break;
    }
  }

  @Override
  protected SystemState handleStateTransitions() {
    var requested = getRequestedState();
    switch (requested) {
      case TO_POSE_PID:
        linearController.reset();
        omegaController.reset();
        break;
      default:
        break;
    }
    return requested;
  }

  private void runVelocity(ChassisSpeeds velocity) {
    var moduleStates = kinematics.toSwerveModuleStates(velocity);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      moduleStates,
      DriveConstants.maxLinearSpeed
    );

    for (int i = 0; i < 4; ++i) swerveModules[i].runState(moduleStates[i]);
  }

  public void setYaw(Angle yaw) {
    gyro.setYaw(yaw);
  }

  private void pidToPose() {
    var robotPose = RobotState.getInstance().getEstimatedPose();
    // A - B = B -> A
    var robotToTarget = driveToPointPose.getTranslation().minus(robotPose.getTranslation());
    double distance = robotPose.getTranslation().getDistance(driveToPointPose.getTranslation());
    var output = Math.min(linearController.calculate(0, distance), DriveConstants.maxLinearSpeed);

    double vw =
        omegaController.calculate(
            robotPose.getRotation().getRadians(), driveToPointPose.getRotation().getRadians());

    var vx = output * robotToTarget.getAngle().getCos();
    var vy = output * robotToTarget.getAngle().getSin();

    if (linearController.atSetpoint() && omegaController.atSetpoint()) {
      setState(SystemState.TELEOP_DRIVE);
			runVelocity(new ChassisSpeeds());
    } else {
      // var robotVelocity = ChassisVelocity(RadiansPerSecond.of(vw), linearVector);
      var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vw, gyroData.yaw);

      Logger.recordOutput("pidToPose/pidLinearOutput", output);
      Logger.recordOutput("pidToPose/pidOmegaOutput", vw);
      Logger.recordOutput("pidToPose/distance", distance);
      Logger.recordOutput("pidToPose/driveToPointPose", driveToPointPose);

      runVelocity(robotVelocity);
    }
  }

  private ChassisSpeeds getControllerSpeeds() {
    final double sX = MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.controllerDeadband);
    final double sY = MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.controllerDeadband);
    double sW = MathUtil.applyDeadband(controller.getRightX(), DriveConstants.controllerDeadband);

    sW = Math.copySign(sW * sW, sW); // heuristic?
    final double sign = FieldConstants.isBlueAlliance() ? -1 : 1;
    final double vX = sX * DriveConstants.maxLinearSpeed * sign;
    final double vY = sY * DriveConstants.maxLinearSpeed * sign;
    final double vW = sW * DriveConstants.maxOmega;

    return ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vW, gyroData.yaw);
  }
}
