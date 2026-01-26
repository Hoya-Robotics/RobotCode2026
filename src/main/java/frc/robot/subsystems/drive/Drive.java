package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.StateSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateSubsystem<frc.robot.subsystems.drive.Drive.SystemState> {
  public enum SystemState {
    IDLE,
    TELEOP_DRIVE,
    TO_POSE_PID,
  };

  // Drive to pose controllers
  private final PIDController linearController =
      new PIDController(
          DriveConstants.driveGains.kp(),
          DriveConstants.driveGains.ki(),
          DriveConstants.driveGains.kd());
  private final PIDController omegaController =
      new PIDController(
          DriveConstants.rotGains.kp(), DriveConstants.rotGains.ki(), DriveConstants.rotGains.kd());

  private Notifier simThread = null;
  private final XboxController controller;
  private Pose2d driveToPointPose = new Pose2d();
  private final SwerveIO io;
  private final ModuleIOInputsAutoLogged[] moduleInputs =
      new ModuleIOInputsAutoLogged[] {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
      };
  private final SwerveIOInputsAutoLogged swerveInputs = new SwerveIOInputsAutoLogged();

  public Drive(SwerveIO io, XboxController controller) {
    this.controller = controller;
    this.io = io;

    io.setTelemetryInputs(swerveInputs);

    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    linearController.setTolerance(DriveConstants.driveTolerance.in(Meters));
    omegaController.setTolerance(DriveConstants.rotateTolerance.in(Radians));

    setState(SystemState.TO_POSE_PID);
    driveToPointPose = new Pose2d(2.0, 2.0, Rotation2d.kZero);
  }

  public void startSimThread(Time simPeriod) {
    SimulatedArena.overrideSimulationTimings(simPeriod, 1);
    simThread =
        new Notifier(
            () -> {
              SimulatedArena.getInstance().simulationPeriodic();
              io.updateSim();
            });
    simThread.startPeriodic(simPeriod.in(Seconds));
  }

  @Override
  public void periodic() {
    io.updateModuleInputs(moduleInputs);

    for (int i = 0; i < 4; ++i) Logger.processInputs("Drive/Module" + i, moduleInputs[i]);
    Logger.processInputs("Drive", swerveInputs);

    Logger.recordOutput("Drive/systemState", getCurrentState());

    if (Constants.getMode() == Mode.SIM) {
      Logger.recordOutput("Drive/simulatedPose", io.getSimPose());
    }

    statePeriodic();
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
    io.applyRequest(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(velocity));
  }

  private void pidToPose() {
    var robotPose = swerveInputs.pose;
    if (Constants.getMode() == Mode.SIM) robotPose = io.getSimPose();
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
      ChassisSpeeds robotVelocity =
          ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vw, swerveInputs.yaw);

      Logger.recordOutput("pidToPose/pidLinearOutput", output);
      Logger.recordOutput("pidToPose/pidOmegaOutput", vw);
      Logger.recordOutput("pidToPose/distance", distance);
      Logger.recordOutput("pidToPose/driveToPointPose", driveToPointPose);

      runVelocity(robotVelocity);
    }
  }

  private ChassisSpeeds getControllerSpeeds() {
    final double sX =
        MathUtil.applyDeadband(controller.getLeftY(), DriveConstants.controllerDeadband);
    final double sY =
        MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.controllerDeadband);
    double sW = MathUtil.applyDeadband(controller.getRightX(), DriveConstants.controllerDeadband);

    sW = Math.copySign(sW * sW, sW); // heuristic?
    final double sign = FieldConstants.isBlueAlliance() ? -1 : 1;
    final double vX = sX * DriveConstants.maxLinearSpeed * sign;
    final double vY = sY * DriveConstants.maxLinearSpeed * sign;
    final double vW = sW * DriveConstants.maxOmega;

    return ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vW, swerveInputs.yaw);
  }
}
