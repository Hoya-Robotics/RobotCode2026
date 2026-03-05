package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private Supplier<Pose2d> simulatedDrivePoseSupplier = () -> Pose2d.kZero;
  private Drive drive;
  private DriveIOInputsAutoLogged driveInputs;

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public void registerDrivetrain(Drive drive) {
    this.drive = drive;
  }

  public void registerSimPoseSupplier(Supplier<Pose2d> supplier) {
    simulatedDrivePoseSupplier = supplier;
  }

  public void resetOdometry(Pose2d pose) {
    if (drive != null) {
      drive.resetOdometry(pose);
    }
  }

  public void updateTurretState(TurretState state) {
    this.turretState = state;
  }

  public void addVisionMeasurement(VisionObservation estimate) {
    if (drive != null) {
      drive.addVisionMeasurement(estimate);
    }
  }

  public void addDriveInputs(DriveIOInputsAutoLogged inputs) {
    this.driveInputs = inputs;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(driveInputs.Speeds, driveInputs.gyroYaw);
  }

  @AutoLogOutput(key = "RobotState/estimatedPose")
  public Pose2d getEstimatedPose() {
    return new Pose2d(driveInputs.Pose.getTranslation(), driveInputs.gyroYaw);
  }

  public Pose2d getSimulatedPose() {
    return simulatedDrivePoseSupplier.get();
  }

  public TurretState getTurretSetpoints(TurretState turretState) {
    var robotPose = getEstimatedPose();
    var speeds = getFieldVelocity();
    var futurePose = robotPose.exp(speeds.toTwist2d(0.003));

    Transform3d robotToTurret = TurretConstants.robotToTurret;
    Rotation2d heading = driveInputs.gyroYaw;

    // v_hat = v + w x r
    double turretVx =
        speeds.vxMetersPerSecond
            + speeds.omegaRadiansPerSecond
                * (robotToTurret.getY() * heading.getCos()
                    - robotToTurret.getX() * heading.getSin());
    double turretVy =
        speeds.vxMetersPerSecond
            + speeds.omegaRadiansPerSecond
                * (robotToTurret.getX() * heading.getCos()
                    - robotToTurret.getY() * heading.getSin());

    Pose2d turretPose =
        futurePose.transformBy(
            new Transform2d(
                robotToTurret.getX(),
                robotToTurret.getY(),
                robotToTurret.getRotation().toRotation2d()));

		return new TurretState(Radians.of(0), Radians.of(0), RadiansPerSecond.of(0));
  }

  public record VisionObservation(Pose2d pose, Vector<N3> stdDevs, double timestamp) {}
  public record TurretState(Angle azimuthAngle, Angle hoodAngle, AngularVelocity launchSpeed) {}
}
