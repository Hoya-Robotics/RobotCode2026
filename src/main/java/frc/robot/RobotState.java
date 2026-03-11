package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.*;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launcherSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private LoggedTunableNumber hoodAngleTuning = new LoggedTunableNumber("hoodAngleDegrees", 0.0);
  private LoggedTunableNumber launcherSpeedTuning = new LoggedTunableNumber("launcherSpeed", 20.0);

  private static final int kMaxIterations = 10;
  private static final double kConvergenceEpsilon = 0.001;

  private Optional<DoubleConsumer> captureRewind = Optional.empty();

  static {
    hoodAngleMap.put(1.5, Rotation2d.fromDegrees(2.0));
    hoodAngleMap.put(2.0, Rotation2d.fromDegrees(6.0));
    hoodAngleMap.put(2.6, Rotation2d.fromDegrees(7.2));
    hoodAngleMap.put(3.0, Rotation2d.fromDegrees(8.25));
    hoodAngleMap.put(3.5, Rotation2d.fromDegrees(10.0));
    hoodAngleMap.put(3.7, Rotation2d.fromDegrees(11.5));
    hoodAngleMap.put(4.0, Rotation2d.fromDegrees(9.75));
    hoodAngleMap.put(4.5, Rotation2d.fromDegrees(15));
    hoodAngleMap.put(5.2, Rotation2d.fromDegrees(18));

    launcherSpeedMap.put(1.5, 28.0);
    launcherSpeedMap.put(2.0, 27.5);
    launcherSpeedMap.put(2.6, 30.0);
    launcherSpeedMap.put(3.0, 30.0);
    launcherSpeedMap.put(3.5, 32.0);
    launcherSpeedMap.put(3.7, 33.0);
    launcherSpeedMap.put(4.0, 34.5);
    launcherSpeedMap.put(4.5, 34.0);
    launcherSpeedMap.put(5.2, 40.0);

    timeOfFlightMap.put(2.0, 9.0 / 8.0);
    timeOfFlightMap.put(2.6, 9.5 / 8.0);
    timeOfFlightMap.put(3.0, 9.5 / 8.0);
    timeOfFlightMap.put(3.5, 1.0);
    timeOfFlightMap.put(3.7, 7.0 / 8.0);
    timeOfFlightMap.put(4.0, 10.5 / 8.0);
  }

  private FuelSim fuelSim;
  private int fuelInHopper = 0;

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

  public void registerFuelSim(FuelSim fuelSim) {
    this.fuelSim = fuelSim;
  }

  public void registerSimPoseSupplier(Supplier<Pose2d> supplier) {
    simulatedDrivePoseSupplier = supplier;
  }

  public void registerRewindCallback(DoubleConsumer callback) {
    captureRewind = Optional.of(callback);
  }

  public void captureRewind(double duration) {
    captureRewind.ifPresent(capture -> capture.accept(duration));
  }

  public void resetOdometry(Pose2d pose) {
    if (drive != null) {
      drive.resetOdometry(pose);
    }
  }

  public void addVisionMeasurement(VisionObservation estimate) {
    boolean autoNeutral =
        DriverStation.isAutonomousEnabled() && FieldConstants.inNeutralZone(getEstimatedPose());
    if (drive != null && !autoNeutral) {
      drive.addVisionMeasurement(estimate);
    }
  }

  public void addDriveInputs(DriveIOInputsAutoLogged inputs) {
    this.driveInputs = inputs;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(driveInputs.Speeds, driveInputs.gyroYaw);
  }

  public FuelSim getFuelSim() {
    return fuelSim;
  }

  public void addFuel() {
    fuelInHopper += 1;
    Logger.recordOutput("RobotState/simFuelCount", fuelInHopper);
  }

  public boolean consumeFuel() {
    boolean hasFuel = fuelInHopper > 0;
    if (hasFuel) fuelInHopper -= 1;
    Logger.recordOutput("RobotState/simFuelCount", fuelInHopper);
    return hasFuel;
  }

  @AutoLogOutput(key = "RobotState/estimatedPose")
  public Pose2d getEstimatedPose() {
    return new Pose2d(driveInputs.Pose.getTranslation(), driveInputs.gyroYaw);
  }

  public Pose2d getSimulatedPose() {
    return simulatedDrivePoseSupplier.get();
  }

  public TurretState resolveTurretTargetting(RobotConfig.TurretTarget target) {
    Logger.recordOutput("Tuning/hubPose", FieldConstants.Hub.getTopCenter());
    switch (target) {
      case ON_THE_MOVE:
        return turretIterativeMovingSetpoint();
      case HUB:
        return getTurretSetpoint();
      case TUNING:
        var setpoint = getTurretSetpoint();
        return new TurretState(
            setpoint.azimuthAngle(),
            Degrees.of(hoodAngleTuning.getAsDouble()),
            RotationsPerSecond.of(launcherSpeedTuning.getAsDouble()));
      case CONSTANT_FORWARD:
        return new TurretState(Radians.of(0.0), Radians.of(0.0), RadiansPerSecond.of(0.0));
      case NEAREST_TAG:
        List<Pose2d> tagPoses =
            FieldConstants.aprilLayout.getTags().stream()
                .map(tag -> AllianceFlip.apply(tag.pose.toPose2d()))
                .toList();
        Pose2d robotPose = getEstimatedPose();
        Translation2d nearestTag = robotPose.nearest(tagPoses).getTranslation();
        Logger.recordOutput("RobotState/tracking/nearestTag", robotPose.nearest(tagPoses));
        return new TurretState(
            nearestTag
                .minus(robotPose.getTranslation())
                .getAngle()
                .minus(driveInputs.gyroYaw)
                .getMeasure(),
            Radians.of(0),
            RotationsPerSecond.of(0.0));
        // case PASSING: // TODO: implement
      default:
        return new TurretState(Radians.of(0), Radians.of(0), RotationsPerSecond.of(0.0));
    }
  }

  public TurretState getTurretSetpoint() {
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
  }

  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
  public TurretState turretIterativeMovingSetpoint() {
    Pose2d robotPose = getEstimatedPose();
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    Translation2d target = FieldConstants.Hub.getTopCenter().toTranslation2d();

    double targetDist = turretPose.getTranslation().getDistance(target);
    Translation2d fieldVelocity =
        rigidPointVelocity(
                driveInputs.Speeds,
                TurretConstants.robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(driveInputs.gyroYaw);

    double lastDist = targetDist;
    double timeOfFlight = timeOfFlightMap.get(targetDist);
    Translation2d adjustedTurretTranslation = turretPose.getTranslation();
    for (int i = 0; i < kMaxIterations; ++i) {
      timeOfFlight = timeOfFlightMap.get(targetDist);
      adjustedTurretTranslation =
          turretPose.getTranslation().plus(fieldVelocity.times(timeOfFlight));
      lastDist = targetDist;
      targetDist = adjustedTurretTranslation.getDistance(target);
      if (Math.abs(targetDist - lastDist) < kConvergenceEpsilon) break;
    }

    Rotation2d futureGyroYaw =
        driveInputs.gyroYaw.plus(
            Rotation2d.fromRadians(driveInputs.Speeds.omegaRadiansPerSecond * timeOfFlight));
    Rotation2d azimuthAngle =
        target.minus(adjustedTurretTranslation).getAngle().minus(futureGyroYaw);
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
}
