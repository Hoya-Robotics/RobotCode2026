package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.RobotConfig.*;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import frc.robot.util.AllianceFlip;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launcherSpeedMap =
      new InterpolatingDoubleTreeMap();
  /*
  private static final InterpolatingDoubleTreeMap launcherVoltageMap =
      new InterpolatingDoubleTreeMap();*/
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private LoggedTunableNumber hoodAngleTuning = new LoggedTunableNumber("hoodAngleDegrees", 0.0);
  /*
  private LoggedTunableNumber launcherVoltageTuning =
      new LoggedTunableNumber("launcherVoltage", 12.0);*/
  private LoggedTunableNumber launcherSpeedTuning = new LoggedTunableNumber("launcherSpeed", 20.0);

  private static final int kMaxIterations = 10;
  private static final double kConvergenceEpsilon = 0.001;

  static {
    hoodAngleMap.put(2.0, Rotation2d.fromDegrees(6.0));
    launcherSpeedMap.put(2.0, 27.5);

    hoodAngleMap.put(2.6, Rotation2d.fromDegrees(7.2));
    launcherSpeedMap.put(2.6, 30.0);

    hoodAngleMap.put(3.0, Rotation2d.fromDegrees(9.0));
    launcherSpeedMap.put(3.0, 31.0);

    hoodAngleMap.put(3.5, Rotation2d.fromDegrees(10.0));
    launcherSpeedMap.put(3.5, 32.0);

    hoodAngleMap.put(4.0, Rotation2d.fromDegrees(9.75));
    launcherSpeedMap.put(4.0, 34.5);
  }

  private Supplier<Pose2d> simulatedDrivePoseSupplier = () -> Pose2d.kZero;
  private Drive drive;
  private DriveIOInputsAutoLogged driveInputs;

  private static RobotState instance;

  private RobotState() {
    AutoLogOutputManager.addObject(this);
  }

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

  public TurretState resolveTurretTargetting(RobotConfig.TurretTarget target) {
    switch (target) {
      case HUB:
        return getShootOnTheMoveTurretSetpoint();
      case TUNING:
        return turretTuning();
      case CONSTANT_FORWARD:
        return new TurretState(Radians.of(0.0), Radians.of(0.0), RadiansPerSecond.of(0.0));
      case NEAREST_TAG:
        List<Pose2d> tagPoses =
            FieldConstants.aprilLayout.getTags().stream()
                .map(tag -> AllianceFlip.apply(tag.pose.toPose2d()))
                .toList();
        Translation2d nearestTag = getEstimatedPose().nearest(tagPoses).getTranslation();
        return new TurretState(
            getMotionAdjustedAzimuth(nearestTag), Radians.of(0), RotationsPerSecond.of(0.0));
      case PASSING: // TODO: implement
      default:
        return new TurretState(Radians.of(0), Radians.of(0), RotationsPerSecond.of(0.0));
    }
  }

  public TurretState turretTuning() {
    Pose2d robotPose = getEstimatedPose();
    Translation2d hubPosition = FieldConstants.Hub.getTopCenter().toTranslation2d();
    Rotation2d azimuth =
        hubPosition.minus(robotPose.getTranslation()).getAngle().minus(driveInputs.gyroYaw);
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    double hubDistance = turretPose.getTranslation().getDistance(hubPosition);
    Logger.recordOutput("Tuning/hubDistance", hubDistance);
    return new TurretState(
        azimuth.getMeasure(),
        hoodAngleMap.get(hubDistance).getMeasure(),
        RotationsPerSecond.of(launcherSpeedMap.get(hubDistance)));
    /*
    return new TurretState(
        azimuth.getMeasure(),
        Degrees.of(hoodAngleTuning.getAsDouble()),
        RotationsPerSecond.of(launcherSpeedTuning.getAsDouble()));*/
  }

  public Angle getMotionAdjustedAzimuth(Translation2d target) {
    Pose2d robotPose = getEstimatedPose();
    Transform3d robotToTurret = TurretConstants.robotToTurret;
    Translation2d turret =
        new Pose3d(robotPose).transformBy(robotToTurret).toPose2d().getTranslation();
    Translation2d turretToFieldVelocity =
        rigidPointVelocity(driveInputs.Speeds, robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(driveInputs.gyroYaw);
    turret = turret.plus(turretToFieldVelocity.times(TurretConstants.azimuthLatencyCompensation));
    return target.minus(turret).getAngle().minus(driveInputs.gyroYaw).getMeasure();
  }

  @Deprecated
  public TurretState getTurretSetpoint() {
    Pose2d robotPose = getEstimatedPose();
    Translation2d target = FieldConstants.Hub.getTopCenter().toTranslation2d();
    double distance = robotPose.getTranslation().getDistance(target);

    Angle robotToTarget =
        target
            .minus(robotPose.getTranslation())
            .getAngle()
            .minus(robotPose.getRotation())
            .getMeasure();
    return new TurretState(
        robotToTarget,
        hoodAngleMap.get(distance).getMeasure(),
        RotationsPerSecond.of(launcherSpeedMap.get(distance)));
  }

  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
  public TurretState getShootOnTheMoveTurretSetpoint() {
    Pose2d robotPose = getEstimatedPose();
    Translation2d target = FieldConstants.Hub.getTopCenter().toTranslation2d();

    Pose2d futurePose = robotPose.exp(driveInputs.Speeds.toTwist2d(0.003));
    Transform3d robotToTurret = TurretConstants.robotToTurret;
    Pose2d turretPose = new Pose3d(futurePose).transformBy(robotToTurret).toPose2d();
    Translation2d turretToFieldVelocity =
        rigidPointVelocity(driveInputs.Speeds, robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(driveInputs.gyroYaw);

    double targetDist = turretPose.getTranslation().getDistance(target);
    double lastDist = targetDist;
    double timeOfFlight = timeOfFlightMap.get(targetDist);
    Translation2d adjustedTurretTranslation = turretPose.getTranslation();
    for (int i = 0; i < kMaxIterations; ++i) {
      timeOfFlight = timeOfFlightMap.get(targetDist);
      adjustedTurretTranslation =
          turretPose.getTranslation().plus(turretToFieldVelocity.times(timeOfFlight));
      lastDist = targetDist;
      targetDist = adjustedTurretTranslation.getDistance(target);
      if (Math.abs(targetDist - lastDist) < kConvergenceEpsilon) break;
    }

    Rotation2d azimuthAngle =
        target.minus(adjustedTurretTranslation).getAngle().minus(driveInputs.gyroYaw);
    return new TurretState(
        azimuthAngle.getMeasure(),
        hoodAngleMap.get(targetDist).getMeasure(),
        RotationsPerSecond.of(launcherSpeedMap.get(targetDist)));
  }

  private static Translation2d rigidPointVelocity(ChassisSpeeds speeds, Translation2d r) {
    return new Translation2d(
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * r.getY(),
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r.getX());
  }

  public record VisionObservation(Pose2d pose, Vector<N3> stdDevs, Time timestamp) {
    public double timestampSeconds() {
      return timestamp.in(Seconds);
    }
  }

  public record TurretState(Angle azimuthAngle, Angle hoodAngle, AngularVelocity launcherSpeed) {}
  // public record TurretState(Angle azimuthAngle, Angle hoodAngle, double launchVoltage) {}
}
