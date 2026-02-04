package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.*;
import frc.robot.RobotState;
import frc.robot.RobotState.*;
import frc.robot.util.AllianceFlip;
import frc.robot.util.StateSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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

  private SwerveDriveSimulation sim = null;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  private PIDController linearController =
      new PIDController(
          DriveConstants.toPoseLinearGains.kp(),
          DriveConstants.toPoseLinearGains.ki(),
          DriveConstants.toPoseLinearGains.kd());
  private PIDController omegaController =
      new PIDController(
          DriveConstants.toPoseOmegaGains.kp(),
          DriveConstants.toPoseOmegaGains.ki(),
          DriveConstants.toPoseOmegaGains.kd());

  private Pose2d targetDrivePose = null;

  public Drive(XboxController controller, GyroIO gyro, ModuleIO[] moduleIOs) {
    this.driveController = controller;
    this.gyro = gyro;
    for (int i = 0; i < 4; ++i) {
      this.modules[i] = new Module(i, moduleIOs[i]);
    }

    linearController.setTolerance(DriveConstants.toPoseLinearTolerance);
    omegaController.setTolerance(DriveConstants.toPoseThetaTolerance);
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    setState(DriveState.IDLE);
  }

  public void setSimDrivetrain(SwerveDriveSimulation sim) {
    this.sim = sim;
  }

  public static Drive empty() {
    return new Drive(null, new GyroIO() {}, new ModuleIO[4]);
  }

  public static Drive simulatedDrive(XboxController controller) {
    var startingPose =
        AllianceFlip.apply(
            new Pose2d(
                FieldConstants.Hub.nearFace.getX() - 0.5,
                FieldConstants.fieldWidth / 2.0,
                Rotation2d.fromDegrees(225)));
    var sim = new SwerveDriveSimulation(SimConstants.mapleSwerveConfig, startingPose);
    SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    RobotState.getInstance().hardSetOdometry(startingPose);
    RobotState.getInstance().hardSetKalmanPose(startingPose);
    RobotState.getInstance().addSimPoseSupplier(sim::getSimulatedDriveTrainPose);

    var instance =
        new Drive(
            controller,
            new GyroIOSim(sim.getGyroSimulation()),
            new ModuleIO[] {
              new ModuleIOSim(sim.getModules()[0]),
              new ModuleIOSim(sim.getModules()[1]),
              new ModuleIOSim(sim.getModules()[2]),
              new ModuleIOSim(sim.getModules()[3])
            });
    sim.getGyroSimulation().setRotation(startingPose.getRotation());
    instance.setSimDrivetrain(sim);

    return instance;
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroData);
    Logger.processInputs("Drive/Gyro", gyroData);
    Logger.recordOutput("Drive/systemState", getCurrentState());

    var chassisSpeeds = getChassisSpeeds();
    Logger.recordOutput("Drive/chassisSpeeds", chassisSpeeds);
    Logger.recordOutput(
        "Drive/linearSpeed",
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));

    if (DriverStation.isTeleop() && getCurrentState() == DriveState.IDLE) {
      setState(DriveState.TELEOP);
    }

    var modulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; ++i) {
      modules[i].periodic();
      modulePositions[i] = modules[i].getPosition();
    }

    RobotState.getInstance()
        .addOdometryObservation(
            new OdometryObservation(
                getChassisSpeeds(), modulePositions, gyroData.yaw, Timer.getFPGATimestamp()));

    statePeriodic();

    for (var m : modules) m.applyOutputs();
  }

  @Override
  public void simulationPeriodic() {
    Logger.recordOutput("Drive/simulatedPose", sim.getSimulatedDriveTrainPose());
  }

  public void driveToPose(Pose2d target) {
    this.targetDrivePose = target;
    setState(DriveState.TO_POSE);
  }

  public void runSetpoint(ChassisSpeeds speeds) {
    var discrete = ChassisSpeeds.discretize(speeds, 0.02);
    var moduleStates = kinematics.toSwerveModuleStates(discrete);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.maxDriveSpeedMps);

    for (int i = 0; i < 4; ++i) {
      modules[i].runSetpoint(moduleStates[i]);
    }
  }

  public void xBrake() {
    for (int i = 0; i < 4; ++i) {
      modules[i].runSetpoint(DriveConstants.xBrakeStates[i]);
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
        var robotPose = RobotState.getInstance().getEstimatedRobotPose();
        runSetpoint(getPidToPoseSetpoint(robotPose));
        break;
      case IDLE:
        xBrake();
        break;
    }
  }

  private ChassisSpeeds getPidToPoseSetpoint(Pose2d robotPose) {
    var robotToTarget = targetDrivePose.getTranslation().minus(robotPose.getTranslation());
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
                robotPose.getRotation().getRadians(), targetDrivePose.getRotation().getRadians()),
            -DriveConstants.maxRotationSpeedRps,
            DriveConstants.maxRotationSpeedRps);

    Logger.recordOutput("Drive/ToPose/TargetPose", targetDrivePose);
    Logger.recordOutput("Drive/ToPose/linearError", distance);
    Logger.recordOutput("Drive/ToPose/headingDeg", heading.getDegrees());
    Logger.recordOutput("Drive/ToPose/linearOutput", linearOutput);
    Logger.recordOutput("Drive/ToPose/OmegaOutput", omega);

    /*
    if (linearController.atSetpoint() && omegaController.atSetpoint()) {
      setState(DriveState.IDLE);
      return new ChassisSpeeds();
    }*/

    return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyroData.yaw);
  }

  private ChassisSpeeds getControllerSpeeds() {
    double sx = -driveController.getLeftY();
    double sy = -driveController.getLeftX();
    double omega = -driveController.getRightX();

    omega = MathUtil.applyDeadband(omega, DriveConstants.controllerDeadband);
    omega = omega * omega * Math.signum(omega); // heuristic

    var heading = new Rotation2d(sx, sy);
    var magnitude = MathUtil.applyDeadband(Math.hypot(sx, sy), DriveConstants.controllerDeadband);
    magnitude = magnitude * magnitude; // heuristic

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        magnitude * heading.getCos(), magnitude * heading.getSin(), omega, gyroData.yaw);
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; ++i) moduleStates[i] = modules[i].getState();

    return kinematics.toChassisSpeeds(moduleStates);
  }
}
