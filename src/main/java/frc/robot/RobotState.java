package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launcherSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final int kMaxIterations = 10;
  private static final double kConvergenceEpsilon = 0.001;

  static {
    // TODO: measure
  }

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

  public void addVisionMeasurement(VisionObservation estimate) {
    if (drive != null) {
      drive.addVisionMeasurement(estimate);
    }
  }

  public void addDriveInputs(DriveIOInputsAutoLogged inputs) {
    this.driveInputs = inputs;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(driveInputs.Speeds, driveInputs.gyroYaw);
  }

  @AutoLogOutput(key = "RobotState/estimatedPose")
  public Pose2d getEstimatedPose() {
    return new Pose2d(driveInputs.Pose.getTranslation(), driveInputs.gyroYaw);
  }

  public Pose2d getSimulatedPose() {
    return simulatedDrivePoseSupplier.get();
  }

  public TurretState getTurretSetpoints(TurretState turretState) {
    Pose2d robotPose = getEstimatedPose();
    ChassisSpeeds speeds = driveInputs.Speeds;
    Pose2d futurePose = robotPose.exp(speeds.toTwist2d(0.003));
    Transform3d robotToTurret = TurretConstants.robotToTurret;

    // Phase Shifted Turret Pose
    Pose2d turretPose =
        futurePose.transformBy(
            new Transform2d(
                robotToTurret.getX(),
                robotToTurret.getY(),
                robotToTurret.getRotation().toRotation2d()));
    Translation2d target = FieldConstants.hubCenterPoint.getTranslation();
    double targetDistance = turretPose.getTranslation().getDistance(target);

    // v_turret = v_robot + w x r_turret
    double r_turretVx =
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * robotToTurret.getY();
    double r_turretVy =
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * robotToTurret.getX();
    Translation2d f_turretV =
        new Translation2d(r_turretVx, r_turretVy).rotateBy(driveInputs.gyroYaw);
    double f_turretVx = f_turretV.getX();
    double f_turretVy = f_turretV.getY();

    double timeOfFlight = timeOfFlightMap.get(targetDistance);
    Pose2d lookaheadPose = turretPose;
    double lookaheadTargetDist = targetDistance;

    // Iterative lookahead adjust
    for (int i = 0; i < kMaxIterations; ++i) {
      double prevDist = lookaheadTargetDist;
      timeOfFlight = timeOfFlightMap.get(lookaheadTargetDist);
      lookaheadPose =
          new Pose2d(
              turretPose
                  .getTranslation()
                  .plus(new Translation2d(f_turretVx * timeOfFlight, f_turretVy * timeOfFlight)),
              turretPose.getRotation());
      lookaheadTargetDist = lookaheadPose.getTranslation().getDistance(target);
      if (Math.abs(lookaheadTargetDist - prevDist) < kConvergenceEpsilon) break;
    }

    // Calculate remaining parameters
    Rotation2d hoodAngle = hoodAngleMap.get(lookaheadTargetDist);
    double launchSpeed = launcherSpeedMap.get(lookaheadTargetDist);
    Rotation2d azimuth = target.minus(lookaheadPose.getTranslation()).getAngle();

    return new TurretState(
        azimuth.getMeasure(), hoodAngle.getMeasure(), RadiansPerSecond.of(launchSpeed));
  }

  public record VisionObservation(Pose2d pose, Vector<N3> stdDevs, Time timestamp) {
    public double timestampSeconds() {
      return timestamp.in(Seconds);
    }
  }

  public record TurretState(Angle azimuthAngle, Angle hoodAngle, AngularVelocity launchSpeed) {}
}
