package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launcherVoltageMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final int kMaxIterations = 10;
  private static final double kConvergenceEpsilon = 0.001;

  static {
    hoodAngleMap.put(3.0, Rotation2d.fromDegrees(7.5));
    launcherVoltageMap.put(3.0, 10.0);

    hoodAngleMap.put(2.5, Rotation2d.fromDegrees(5.5));
    launcherVoltageMap.put(2.5, 10.0);

    hoodAngleMap.put(2.0, Rotation2d.fromDegrees(6.5));
    launcherVoltageMap.put(2.0, 10.5);

    hoodAngleMap.put(1.74, Rotation2d.fromDegrees(2.5));
    launcherVoltageMap.put(1.74, 10.25);
  }

  private Supplier<Pose2d> simulatedDrivePoseSupplier = () -> Pose2d.kZero;
  private Drive drive;
  private DriveIOInputsAutoLogged driveInputs;

  // Hub-relative vision state for turret aiming
  private HubObservation latestHubObservation = null;

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

  public void addHubObservation(HubObservation observation) {
    this.latestHubObservation = observation;
    Logger.recordOutput("RobotState/hubObservation/tagId", observation.tagId());
    Logger.recordOutput("RobotState/hubObservation/confidence", observation.confidence());
    Logger.recordOutput(
        "RobotState/hubObservation/distance", observation.robotToHub().getTranslation().getNorm());
  }

  public Optional<HubObservation> getLatestHubObservation() {
    if (latestHubObservation == null) {
      return Optional.empty();
    }
    double age = Timer.getFPGATimestamp() - latestHubObservation.timestamp();
    if (age > VisionConstants.hubObservationTimeout) {
      return Optional.empty();
    }
    return Optional.of(latestHubObservation);
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
        robotToTarget, hoodAngleMap.get(distance).getMeasure(), launcherVoltageMap.get(distance));
  }

  public TurretState getShootOnTheMoveTurretSetpoint() {
    Pose2d robotPose = getEstimatedPose();
    Translation2d target =
        VisionConstants.useHubLocalizationBlending
            ? calculateBlendedHubTarget(robotPose)
            : FieldConstants.Hub.getTopCenter().toTranslation2d();

    Pose2d futurePose = robotPose.exp(driveInputs.Speeds.toTwist2d(0.003));
    Transform3d robotToTurret = TurretConstants.robotToTurret;
    Pose2d turretPose = new Pose3d(futurePose).transformBy(robotToTurret).toPose2d();
    Translation2d turretToFieldVelocity =
        new Translation2d(
                driveInputs.Speeds.vxMetersPerSecond
                    - driveInputs.Speeds.omegaRadiansPerSecond
                    + robotToTurret.getY(),
                driveInputs.Speeds.vyMetersPerSecond
                    + driveInputs.Speeds.omegaRadiansPerSecond
                    + robotToTurret.getX())
            .rotateBy(driveInputs.gyroYaw);

    double targetDist = turretPose.getTranslation().getDistance(target);
    double lastDist = targetDist;
    double timeOfFlight = timeOfFlightMap.get(targetDist);
    Translation2d adjustedTurretTranslation = turretPose.getTranslation();
    for (int i = 0; i < kMaxIterations; ++i) {
      timeOfFlight = timeOfFlightMap.get(targetDist);
      adjustedTurretTranslation =
          adjustedTurretTranslation.plus(turretToFieldVelocity.times(timeOfFlight));
      lastDist = targetDist;
      targetDist = adjustedTurretTranslation.getDistance(target);
      if (Math.abs(targetDist - lastDist) < kConvergenceEpsilon) break;
    }

    Rotation2d azimuthAngle =
        target.minus(adjustedTurretTranslation).getAngle().minus(driveInputs.gyroYaw);
    return new TurretState(
        azimuthAngle.getMeasure(),
        hoodAngleMap.get(targetDist).getMeasure(),
        launcherVoltageMap.get(targetDist));
  }

  private Translation2d calculateBlendedHubTarget(Pose2d robotPose) {
    Translation2d globalHubTarget = FieldConstants.Hub.getTopCenter().toTranslation2d();

    Optional<HubObservation> hubObs = getLatestHubObservation();
    if (hubObs.isEmpty()) {
      return globalHubTarget;
    }

    HubObservation obs = hubObs.get();

    Translation2d hubFromVision =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(obs.robotToHub().getX(), obs.robotToHub().getY())
                    .rotateBy(robotPose.getRotation()));

    double blendFactor = MathUtil.clamp(obs.confidence(), 0.0, 1.0);

    Logger.recordOutput("RobotState/hubBlend/factor", blendFactor);
    Logger.recordOutput(
        "RobotState/hubBlend/visionTarget", new Pose2d(hubFromVision, Rotation2d.kZero));
    Logger.recordOutput(
        "RobotState/hubBlend/globalTarget", new Pose2d(globalHubTarget, Rotation2d.kZero));

    return globalHubTarget.interpolate(hubFromVision, blendFactor);
  }

  public record VisionObservation(Pose2d pose, Vector<N3> stdDevs, Time timestamp) {
    public double timestampSeconds() {
      return timestamp.in(Seconds);
    }
  }

  public record HubObservation(Pose3d robotToHub, int tagId, double timestamp, double confidence) {}

  public record TurretState(Angle azimuthAngle, Angle hoodAngle, double launchVoltage) {}
  // public record TurretState(Angle azimuthAngle, Angle hoodAngle, AngularVelocity launchSpeed) {}
}
