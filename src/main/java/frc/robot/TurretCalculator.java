package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.AllianceFlip;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class TurretCalculator {
  public record TurretParameters(
      Angle azimuthAngle, Angle hoodAngle, AngularVelocity launcherSpeed) {}

  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launcherSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingLauncherSpeedMap =
      new InterpolatingDoubleTreeMap();

  private static LoggedTunableNumber hoodAngleTuning =
      new LoggedTunableNumber("hoodAngleDegrees", 0.0);
  private static LoggedTunableNumber launcherSpeedTuning =
      new LoggedTunableNumber("launcherSpeed", 20.0);

  private static final int kMaxIterations = 10;
  private static final double kConvergenceEpsilon = 0.001;

  static {
    passingHoodAngleMap.put(0.0, Rotation2d.fromDegrees(0.0));
    passingLauncherSpeedMap.put(0.0, 0.0);

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

  public static TurretParameters calculateSetpoints(
      RobotConfig.TurretTarget target, Angle currentAzimuthAngle) {
    Logger.recordOutput("Tuning/hubPose", FieldConstants.Hub.getTopCenter());
    Translation2d hubPosition = FieldConstants.Hub.getTopCenter().toTranslation2d();
    switch (target) {
      case PASSING:
        return getStationarySetpoint(getPassingTarget());
      case ON_THE_MOVE:
        return turretIterativeMovingSetpoint();
      case HUB:
        return getStationarySetpoint(hubPosition);
      case TUNING:
        var setpoint = getStationarySetpoint(hubPosition);
        return new TurretParameters(
            setpoint.azimuthAngle(),
            Degrees.of(hoodAngleTuning.getAsDouble()),
            RotationsPerSecond.of(launcherSpeedTuning.getAsDouble()));
      case CONSTANT_FORWARD:
        return new TurretParameters(Radians.of(0.0), Radians.of(0.0), RadiansPerSecond.of(0.0));
      case NEAREST_TAG:
        List<Pose2d> tagPoses =
            FieldConstants.aprilLayout.getTags().stream()
                .map(tag -> AllianceFlip.apply(tag.pose.toPose2d()))
                .toList();
        Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
        Translation2d nearestTag = robotPose.nearest(tagPoses).getTranslation();
        Logger.recordOutput("RobotState/tracking/nearestTag", robotPose.nearest(tagPoses));
        return new TurretParameters(
            nearestTag
                .minus(robotPose.getTranslation())
                .getAngle()
                .minus(robotPose.getRotation())
                .getMeasure(),
            Radians.of(0),
            RotationsPerSecond.of(0.0));
      default:
        return new TurretParameters(Radians.of(0), Radians.of(0), RotationsPerSecond.of(0.0));
    }
  }

  private static Angle calculateAzimuthAngle(Angle fieldRelativeTargetAngle, Angle currentAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    double angle =
        MathUtil.inputModulus(
            new Rotation2d(fieldRelativeTargetAngle).minus(robotPose.getRotation()).getRotations(),
            -0.5,
            0.5);
    double current = currentAngle.in(Rotations);
    if (current > 0 && angle + 1 <= TurretConstants.maxAzimuthAngle.in(Rotations)) angle += 1;
    if (current < 0 && angle - 1 >= -TurretConstants.maxAzimuthAngle.in(Rotations)) angle -= 1;
    return Rotations.of(angle);
  }

  private static final double xPassTarget = Units.inchesToMeters(37);
  private static final double yPassTarget = Units.inchesToMeters(65);

  private static Translation2d getPassingTarget() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    double flippedY = AllianceFlip.apply(robotPose).getY();
    boolean mirror = flippedY > FieldConstants.fieldWidth / 2.0;
    return AllianceFlip.apply(
        new Translation2d(
            xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));
  }

  private static TurretParameters getStationarySetpoint(Translation2d target) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Rotation2d azimuth =
        target.minus(robotPose.getTranslation()).getAngle().minus(robotPose.getRotation());
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    double hubDistance = turretPose.getTranslation().getDistance(target);
    Logger.recordOutput("Tuning/hubDistance", hubDistance);
    return new TurretParameters(
        azimuth.getMeasure(),
        hoodAngleMap.get(hubDistance).getMeasure(),
        RotationsPerSecond.of(launcherSpeedMap.get(hubDistance)));
  }

  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
  private static TurretParameters turretIterativeMovingSetpoint() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    Translation2d target = FieldConstants.Hub.getTopCenter().toTranslation2d();

    double targetDist = turretPose.getTranslation().getDistance(target);
    Translation2d fieldVelocity =
        rigidPointVelocity(
                RobotState.getInstance().getRobotVelocity(),
                TurretConstants.robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(robotPose.getRotation());

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
        robotPose
            .getRotation()
            .plus(
                Rotation2d.fromRadians(
                    RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond
                        * timeOfFlight));
    Rotation2d azimuthAngle =
        target.minus(adjustedTurretTranslation).getAngle().minus(futureGyroYaw);
    return new TurretParameters(
        azimuthAngle.getMeasure(),
        hoodAngleMap.get(targetDist).getMeasure(),
        RotationsPerSecond.of(launcherSpeedMap.get(targetDist)));
  }

  private static Translation2d rigidPointVelocity(ChassisSpeeds speeds, Translation2d r) {
    return new Translation2d(
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * r.getY(),
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r.getX());
  }
}
