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
import frc.robot.util.PolynomialRegression;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class TurretCalculator {
  public record TurretParameters(
      Angle azimuthAngle, Angle hoodAngle, AngularVelocity launcherSpeed) {}

  private static final PolynomialRegression hoodAngleFn =
      new PolynomialRegression(
          new double[][] {
            {1.5, 2.0},
            {2.0, 6.0},
            {3.0, 8.25},
            {3.5, 10.0},
            {4.5, 15.0}
          },
          2);
  private static final PolynomialRegression launcherSpeedFn =
      new PolynomialRegression(
          new double[][] {
            {1.5, 28.0},
            {2.0, 27.5},
            {3.0, 30.0},
            {3.5, 32.0},
            {4.5, 34.0}
          },
          2);

  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launcherSpeedMap =
      new InterpolatingDoubleTreeMap();
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

    // Experimental LUTs
    newHoodLUT.put(1.5, Rotation2d.fromDegrees(5.0));
    newHoodLUT.put(2.0, Rotation2d.fromDegrees(8.5));
    newHoodLUT.put(2.6, Rotation2d.fromDegrees(15.0));
    newHoodLUT.put(3.0, Rotation2d.fromDegrees(18.5));
    newHoodLUT.put(3.5, Rotation2d.fromDegrees(19.75));
    newHoodLUT.put(4.0, Rotation2d.fromDegrees(22.25));
    // newHoodLUT.put(4.5, Rotation2d.fromDegrees(21.25));
    // newHoodLUT.put(5.2, Rotation2d.fromDegrees(18));

    newLauncherLUT.put(1.5, 26.75);
    newLauncherLUT.put(2.0, 27.0);
    newLauncherLUT.put(2.6, 29.0);
    newLauncherLUT.put(3.0, 29.25);
    newLauncherLUT.put(3.5, 31.5);
    newLauncherLUT.put(4.0, 33.5);
    // newLauncherLUT.put(4.5, 33.5);

    // Current LUTs
    hoodAngleMap.put(1.5, Rotation2d.fromDegrees(2.0));
    hoodAngleMap.put(2.0, Rotation2d.fromDegrees(6.0));
    hoodAngleMap.put(2.6, Rotation2d.fromDegrees(7.2));
    hoodAngleMap.put(3.0, Rotation2d.fromDegrees(8.25));
    hoodAngleMap.put(3.5, Rotation2d.fromDegrees(10.0));
    hoodAngleMap.put(3.7, Rotation2d.fromDegrees(11.5));
    hoodAngleMap.put(4.0, Rotation2d.fromDegrees(12));
    hoodAngleMap.put(4.5, Rotation2d.fromDegrees(15));

    launcherSpeedMap.put(1.5, 28.0);
    launcherSpeedMap.put(2.0, 27.5);
    launcherSpeedMap.put(2.6, 30.0);
    launcherSpeedMap.put(3.0, 30.0);
    launcherSpeedMap.put(3.5, 32.0);
    launcherSpeedMap.put(3.7, 33.0);
    launcherSpeedMap.put(4.0, 33.5);
    launcherSpeedMap.put(4.5, 34.0);
    launcherSpeedMap.put(5.2, 40.0);

    // TOF Data (needs to be retested)
    timeOfFlightMap.put(2.0, 9.0 / 8.0);
    timeOfFlightMap.put(2.6, 9.5 / 8.0);
    timeOfFlightMap.put(3.0, 9.5 / 8.0);
    timeOfFlightMap.put(3.5, 1.0);
    timeOfFlightMap.put(3.7, 7.0 / 8.0);
    timeOfFlightMap.put(4.0, 10.5 / 8.0);
  }

  private static final TurretParameters FRONT_OF_HUB_PARAMS =
      new TurretParameters(Rotations.of(0.0), Degrees.of(2.0), RotationsPerSecond.of(28.0));

  public static TurretParameters calculateSetpoints(
      RobotConfig.TurretTarget target, Angle currentAzimuthAngle) {
    Logger.recordOutput("Tuning/hubPose", FieldConstants.Hub.getTopCenter());
    Translation2d hubPosition = FieldConstants.Hub.getTopCenter().toTranslation2d();
    switch (target) {
      case FRONT_OF_HUB:
        return FRONT_OF_HUB_PARAMS;
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
      case NEAREST_TAG:
        List<Pose2d> tagPoses =
            FieldConstants.aprilLayout.getTags().stream()
                .map(tag -> AllianceFlip.apply(tag.pose.toPose2d()))
                .toList();
        Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
        Translation2d nearestTag = robotPose.nearest(tagPoses).getTranslation();
        Logger.recordOutput("RobotState/tracking/nearestTag", robotPose.nearest(tagPoses));
        return new TurretParameters(
            calculateAzimuthAngle(
                nearestTag
                    .minus(robotPose.getTranslation())
                    .getAngle()
                    .minus(robotPose.getRotation())
                    .getMeasure(),
                currentAzimuthAngle),
            Radians.of(0),
            RotationsPerSecond.of(0.0));
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
        newHoodLUT.get(hubDistance).getMeasure(),
        RotationsPerSecond.of(newLauncherLUT.get(hubDistance) - 0.75));
  }

  // https://github.com/FRC3161/Rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/Drive/CommandSwerveDrivetrain.java#L144
  /*
  private static TurretParameters turretSOFTSetpoint() {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
  }*/

  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
  private static TurretParameters turretIterativeMovingSetpoint(Angle currentAzimuthAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    // TODO: phase shift?? interpolate robot pose into future with .exp() for latent error
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    Translation2d target = FieldConstants.Hub.getTopCenter().toTranslation2d();

    double distance = turretPose.getTranslation().getDistance(target);
    /*
    Translation2d fieldVelocity =
        rigidPointVelocity(
                RobotState.getInstance().getRobotVelocity(),
                TurretConstants.robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(robotPose.getRotation());*/
    ChassisSpeeds fieldSpeeds = RobotState.getInstance().getFieldVelocity();

    double timeOfFlight = timeOfFlightMap.get(distance);
    double lastDist;
    for (int i = 0; i < kMaxIterations; ++i) {
      target =
          target.minus(
              new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond)
                  .times(timeOfFlight));
      lastDist = distance;
      distance = turretPose.getTranslation().getDistance(target);
      if (Math.abs(distance - lastDist) < kConvergenceEpsilon) break;
      timeOfFlight = timeOfFlightMap.get(distance);
    }

    Rotation2d azimuthAngle =
        target.minus(turretPose.getTranslation()).getAngle().minus(robotPose.getRotation());
    return new TurretParameters(
        calculateAzimuthAngle(azimuthAngle.getMeasure(), currentAzimuthAngle),
        hoodAngleMap.get(distance).getMeasure(),
        RotationsPerSecond.of(launcherSpeedMap.get(distance)));
  }

  private static Translation2d rigidPointVelocity(ChassisSpeeds speeds, Translation2d r) {
    return new Translation2d(
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * r.getY(),
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r.getX());
  }
}
