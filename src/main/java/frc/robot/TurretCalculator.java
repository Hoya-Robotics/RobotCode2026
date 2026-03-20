package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    // Passing hood angles
    passingHoodAngleMap.put(3.5, Rotation2d.fromDegrees(11.0));
    passingHoodAngleMap.put(4.0, Rotation2d.fromDegrees(12.5));
    passingHoodAngleMap.put(4.5, Rotation2d.fromDegrees(13.75));

    // Passing launcher angles
    passingLauncherSpeedMap.put(3.5, 24.0);
    passingLauncherSpeedMap.put(4.0, 25.3);
    passingLauncherSpeedMap.put(4.5, 27.5);

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

  private static double hoodRegression(double x) {
    return 4.0 * x - 2.0;
  }

  private static double shotRegression(double x) {
    return 2.27 * x + 22.3;
  }

  private static double tofRegression(double x) {
    return 0.076 * x + 0.982;
  }

  public static TurretParameters calculateSetpoints(
      RobotConfig.TurretTarget trackingTarget, Angle currentAzimuthAngle) {
    boolean passing = FieldConstants.inNeutralZone(RobotState.getInstance().getEstimatedPose());
    Translation2d target =
        AllianceFlip.apply(
            passing ? getPassingTarget() : FieldConstants.hubCenter.toTranslation2d());
    Logger.recordOutput(
        "TurretCalculator/target", new Pose3d(new Translation3d(target), Rotation3d.kZero));
    switch (trackingTarget) {
      case DEFAULT:
        return turretIterativeMovingSetpoint(target, passing, currentAzimuthAngle);
      case TUNING:
        var setpoint = turretIterativeMovingSetpoint(target, passing, currentAzimuthAngle);
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

  private static Translation2d getPassingTarget() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Distance flippedY = AllianceFlip.apply(robotPose).getMeasureY();
    boolean mirror = flippedY.gt(FieldConstants.fieldWidth.div(2.0));
    return new Translation2d(
        FieldConstants.passingX,
        mirror
            ? FieldConstants.fieldWidth.minus(FieldConstants.passingY)
            : FieldConstants.passingY);
  }

  private static TurretParameters getStationarySetpoint(
      Translation2d target, boolean passing, Angle currentAzimuthAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Rotation2d azimuth =
        target.minus(robotPose.getTranslation()).getAngle().minus(robotPose.getRotation());
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    double hubDistance = turretPose.getTranslation().getDistance(target);
    Logger.recordOutput("Tuning/hubDistance", hubDistance);
    return new TurretParameters(
        calculateAzimuthAngle(azimuth.getMeasure(), currentAzimuthAngle),
        passing
            ? passingHoodAngleMap.get(hubDistance).getMeasure()
            : Degrees.of(hoodRegression(hubDistance)),
        RotationsPerSecond.of(
            passing ? passingLauncherSpeedMap.get(hubDistance) : shotRegression(hubDistance)));
  }

  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
  private static TurretParameters turretIterativeMovingSetpoint(
      Translation2d target, boolean passing, Angle currentAzimuthAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    Translation2d fieldVelocity =
        rigidPointVelocity(
                RobotState.getInstance().getRobotVelocity(),
                TurretConstants.robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(robotPose.getRotation());
    if (fieldVelocity.getNorm() < 0.25)
      return getStationarySetpoint(target, passing, currentAzimuthAngle);
    double distance = turretPose.getTranslation().getDistance(target);
    double tof = tofRegression(distance);
    for (int i = 0; i < kMaxIterations; ++i) {
      Translation2d futureTurretPos = turretPose.getTranslation().plus(fieldVelocity.times(tof));
      double lastDist = distance;
      distance = futureTurretPos.getDistance(target);
      if (Math.abs(distance - lastDist) < kConvergenceEpsilon) break;
      tof = tofRegression(distance);
    }

    Translation2d aimVector =
        target.minus(turretPose.getTranslation().plus(fieldVelocity.times(tof)));
    Rotation2d azimuthAngle = aimVector.getAngle().minus(robotPose.getRotation());
    return new TurretParameters(
        calculateAzimuthAngle(azimuthAngle.getMeasure(), currentAzimuthAngle),
        passing
            ? passingHoodAngleMap.get(distance).getMeasure()
            : Degrees.of(hoodRegression(distance)),
        RotationsPerSecond.of(
            passing ? passingLauncherSpeedMap.get(distance) : shotRegression(distance)));
  }

  private static Translation2d rigidPointVelocity(ChassisSpeeds speeds, Translation2d r) {
    return new Translation2d(
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * r.getY(),
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r.getX());
  }
}
