package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.StateSubsystem;
import frc.robot.SwerveDynamics.*;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateSubsystem<frc.robot.subsystems.drive.Drive.SystemState> {
  public enum SystemState {
    IDLE,
    TELEOP_DRIVE,
    TO_POSE_PID,
  };

  private final Module[] swerveModules = new Module[4];
  private final GyroIO gyro;

  private final GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();

  private PIDController linearController =
      new PIDController(
          DriveConstants.driveGains.kp(),
          DriveConstants.driveGains.ki(),
          DriveConstants.driveGains.kd());
  /*
   private ProfiledPIDController omegaController =
       new ProfiledPIDController(
           DriveConstants.rotGains.kp(),
           DriveConstants.rotGains.ki(),
           DriveConstants.rotGains.kd(),
           DriveConstants.rotConstraints);
  */
  private PIDController omegaController =
      new PIDController(
          DriveConstants.rotGains.kp(), DriveConstants.rotGains.ki(), DriveConstants.rotGains.kd());
  private Translation2d[] moduleDisplacements = new Translation2d[4];
  private XboxController controller;

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

    driveToPointPose = new Pose2d(1, 1, Rotation2d.k180deg);
    setState(SystemState.TO_POSE_PID);
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

    var robotVelocity = ChassisVelocity.forwardKinematics(swerveModules);
    Logger.recordOutput("Drive/forwardK/velocity", robotVelocity.velocityVector);
    Logger.recordOutput("Drive/forwardK/omega", robotVelocity.omega);
    /*
      RobotState.getInstance()
          .addOdometryObservation(
              new OdometryObservation(Timer.getFPGATimestamp(), gyroData.yaw, moduleDisplacements));
    */

    for (var m : swerveModules) {
      m.periodicAfter();
    }
  }

  @Override
  protected void applyState() {
    switch (getCurrentState()) {
      case TELEOP_DRIVE:
        runChassisRelativeVelocity(getControllerSpeeds());
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
        // omegaController.reset(gyroData.yaw.getRadians(),
        // gyroData.yawVelocity.in(RadiansPerSecond));
        break;
      default:
        break;
    }
    return requested;
  }

  public void setYaw(Angle yaw) {
    gyro.setYaw(yaw);
  }

  private void pidToPose() {
    // var robotPose = RobotState.getInstance().getEstimatedRobotPose();
    var robotPose = RobotState.getInstance().getSimulatedPose();
    // A - B = B -> A
    var robotToTarget = driveToPointPose.getTranslation().minus(robotPose.getTranslation());
    double distance = robotPose.getTranslation().getDistance(driveToPointPose.getTranslation());
    var output = Math.min(linearController.calculate(0, distance), DriveConstants.maxLinearSpeed);

    double vw =
        omegaController.calculate(
            robotPose.getRotation().getRadians(), driveToPointPose.getRotation().getRadians());

    var linearVector = robotToTarget.times(output / robotToTarget.getNorm());
    var vx = output * robotToTarget.getAngle().getCos();
    var vy = output * robotToTarget.getAngle().getSin();

    if (linearController.atSetpoint() && omegaController.atSetpoint()) {
      setState(SystemState.TELEOP_DRIVE);
      runChassisRelativeVelocity(new ChassisVelocity());
    } else {
      // var robotVelocity = ChassisVelocity(RadiansPerSecond.of(vw), linearVector);
      var robotVelocity = ChassisVelocity.fromFieldRelative(vx, vy, vw, gyroData.yaw);

      Logger.recordOutput("pidToPose/pidLinearOutput", output);
      Logger.recordOutput("pidToPose/pidOmegaOutput", vw);
      Logger.recordOutput("pidToPose/robotToTarget", robotToTarget);
      Logger.recordOutput("pidToPose/distance", distance);
      Logger.recordOutput("pidToPose/driveToPointPose", driveToPointPose);
      Logger.recordOutput("pidToPose/outputLinear", linearVector);

      runChassisRelativeVelocity(robotVelocity);
    }
  }

  private static final double CONTROLLER_DEADBAND = 0.1;

  private ChassisVelocity getControllerSpeeds() {
    final double sX = MathUtil.applyDeadband(controller.getLeftY(), CONTROLLER_DEADBAND);
    final double sY = MathUtil.applyDeadband(controller.getLeftX(), CONTROLLER_DEADBAND);
    double sW = MathUtil.applyDeadband(controller.getRightX(), CONTROLLER_DEADBAND);

    sW = Math.copySign(sW * sW, sW); // heuristic?
    final double sign = FieldConstants.isBlueAlliance() ? -1 : 1;
    final double vX = sX * DriveConstants.maxLinearSpeed * sign;
    final double vY = sY * DriveConstants.maxLinearSpeed * sign;
    final double vW = sW * DriveConstants.maxOmega;

    return ChassisVelocity.fromFieldRelative(vX, vY, vW, gyroData.yaw);
  }

  public final ChassisVelocity getCurrentChassisVelocity() {
    return ChassisVelocity.forwardKinematics(swerveModules);
  }

  private void runChassisRelativeVelocity(ChassisVelocity velocity) {
    var moduleVelocities = velocity.inverseKinematics(swerveModules);
    for (int i = 0; i < 4; ++i) {
      swerveModules[i].runVelocity(moduleVelocities[i]);
    }
  }
}
