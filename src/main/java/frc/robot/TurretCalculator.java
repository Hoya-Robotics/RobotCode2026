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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.AllianceFlip;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class TurretCalculator {
  public record TurretParameters(
      Angle azimuthAngle, Angle hoodAngle, AngularVelocity launcherSpeed) {}

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingTreeMap<Double, Rotation2d> newHoodLUT =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap newLauncherLUT = new InterpolatingDoubleTreeMap();

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

    // Hood angle in
    newHoodLUT.put(1.5, Rotation2d.fromDegrees(5.0));
    newHoodLUT.put(2.0, Rotation2d.fromDegrees(8.5));
    newHoodLUT.put(2.6, Rotation2d.fromDegrees(15.0));
    newHoodLUT.put(3.0, Rotation2d.fromDegrees(18.5));
    newHoodLUT.put(3.5, Rotation2d.fromDegrees(19.75));
    newHoodLUT.put(4.0, Rotation2d.fromDegrees(22.25));

    // Speed in Revolutions Per Second
    newLauncherLUT.put(1.5, 26.75);
    newLauncherLUT.put(2.0, 27.0);
    newLauncherLUT.put(2.6, 29.0);
    newLauncherLUT.put(3.0, 29.25);
    newLauncherLUT.put(3.5, 31.5);
    newLauncherLUT.put(4.0, 33.5);

    // TOF Data (needs to be retested)
    timeOfFlightMap.put(1.5, 0.95);
    timeOfFlightMap.put(2.0, 0.90);
    timeOfFlightMap.put(2.6, 0.94);
    timeOfFlightMap.put(3.0, 1.03);
    timeOfFlightMap.put(3.5, 0.97);
    timeOfFlightMap.put(4.0, 1.07);
  }

  public static TurretParameters calculateSetpoints(
      RobotConfig.TurretTarget target, Angle currentAzimuthAngle) {
    Translation2d hubPosition = AllianceFlip.apply(FieldConstants.hubCenter.toTranslation2d());
    Logger.recordOutput("Tuning/hubPose", hubPosition);
    switch (target) {
      case PASSING:
        return getStationarySetpoint(getPassingTarget(), currentAzimuthAngle);
      case ON_THE_MOVE:
        return turretIterativeMovingSetpoint(currentAzimuthAngle);
      case HUB:
        return getStationarySetpoint(hubPosition, currentAzimuthAngle);
      case TUNING:
        var setpoint = getStationarySetpoint(hubPosition, currentAzimuthAngle);
        return new TurretParameters(
            setpoint.azimuthAngle(),
            Degrees.of(hoodAngleTuning.getAsDouble()),
            RotationsPerSecond.of(launcherSpeedTuning.getAsDouble()));
      case CONSTANT_FORWARD:
        return new TurretParameters(Rotations.of(0.5), Radians.of(0.0), RadiansPerSecond.of(0.0));
      default:
        return new TurretParameters(Radians.of(0), Radians.of(0), RotationsPerSecond.of(0.0));
    }
  }

  private static Angle calculateAzimuthAngle(Angle angle, Angle currentAngle) {
    double targetRotations = angle.in(Rotations);
    double currentRotations = currentAngle.in(Rotations);
    double diff = targetRotations - currentRotations;
    // Closest path
    diff = MathUtil.inputModulus(diff, -0.5, 0.5);
    double closestTarget = currentRotations + diff;
    // Wrapping
    if (closestTarget > TurretConstants.maxAzimuthAngle.in(Rotations)) {
      closestTarget -= 1;
    }
    if (closestTarget < TurretConstants.minAzimuthAngle.in(Rotations)) {
      closestTarget += 1;
    }

    return Rotations.of(closestTarget);
  }

  private static final Distance xPassTarget = Inches.of(37);
  private static final Distance yPassTarget = Inches.of(65);

  private static Translation2d getPassingTarget() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Distance flippedY = AllianceFlip.apply(robotPose).getMeasureY();
    boolean mirror = flippedY.gt(FieldConstants.fieldWidth.div(2.0));
    return AllianceFlip.apply(
        new Translation2d(
            xPassTarget, mirror ? FieldConstants.fieldWidth.minus(yPassTarget) : yPassTarget));
  }

  private static TurretParameters getStationarySetpoint(
      Translation2d target, Angle currentAzimuthAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Rotation2d azimuth =
        target.minus(robotPose.getTranslation()).getAngle().minus(robotPose.getRotation());
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    double hubDistance = turretPose.getTranslation().getDistance(target);
    Logger.recordOutput("Tuning/hubDistance", hubDistance);
    return new TurretParameters(
        calculateAzimuthAngle(azimuth.getMeasure(), currentAzimuthAngle),
        // newHoodLUT.get(hubDistance).getMeasure(),
        Degrees.of(4.0 * hubDistance - 2.0),
        // RotationsPerSecond.of(newLauncherLUT.get(hubDistance) - 1.65));
        RotationsPerSecond.of(2.27 * hubDistance + 22.3));
  }

  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
  private static TurretParameters turretIterativeMovingSetpoint(Angle currentAzimuthAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    Translation2d originalTarget = AllianceFlip.apply(FieldConstants.hubCenter.toTranslation2d());
    Translation2d fieldVelocity =
        rigidPointVelocity(
                RobotState.getInstance().getRobotVelocity(),
                TurretConstants.robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(robotPose.getRotation());
    if (fieldVelocity.getNorm() < 0.25)
      return getStationarySetpoint(originalTarget, currentAzimuthAngle);
    double distance = turretPose.getTranslation().getDistance(originalTarget);
    double tof = timeOfFlightMap.get(distance) + 0.2;
    for (int i = 0; i < kMaxIterations; ++i) {
      Translation2d futureTurretPos = turretPose.getTranslation().plus(fieldVelocity.times(tof));
      double lastDist = distance;
      distance = futureTurretPos.getDistance(originalTarget);
      if (Math.abs(distance - lastDist) < kConvergenceEpsilon) break;
      tof = timeOfFlightMap.get(distance) + 0.2;
    }

    Translation2d aimVector =
        originalTarget.minus(turretPose.getTranslation().plus(fieldVelocity.times(tof)));
    Rotation2d azimuthAngle = aimVector.getAngle().minus(robotPose.getRotation());
    return new TurretParameters(
        calculateAzimuthAngle(azimuthAngle.getMeasure(), currentAzimuthAngle),
        newHoodLUT.get(distance).getMeasure(),
        RotationsPerSecond.of(newLauncherLUT.get(distance) - 1.65));
  }

  private static Translation2d rigidPointVelocity(ChassisSpeeds speeds, Translation2d r) {
    return new Translation2d(
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * r.getY(),
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r.getX());
  }
}
