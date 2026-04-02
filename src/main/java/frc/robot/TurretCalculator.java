package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
import org.littletonrobotics.junction.Logger;

public class TurretCalculator {
  public record TurretParameters(
      Angle azimuthAngle,
      AngularVelocity azimuthVelocity,
      Angle hoodAngle,
      AngularVelocity flywheelSpeed) {}

  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingTreeMap<Double, Rotation2d> PASSING_HOODMAP =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap PASSING_FLYWHEELMAP =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap PASSING_TOFMAP = new InterpolatingDoubleTreeMap();

  private static final int kMaxIterations = 10;
  private static final double kConvergenceEpsilon = 0.001;

  static {
    // Passing hood angles
    PASSING_HOODMAP.put(6.0, Rotation2d.fromDegrees(36.0));
    PASSING_HOODMAP.put(5.0, Rotation2d.fromDegrees(32.0));
    PASSING_HOODMAP.put(4.0, Rotation2d.fromDegrees(29.0));

    // Passing launcher angles
    PASSING_FLYWHEELMAP.put(6.0, 30.0);
    PASSING_FLYWHEELMAP.put(5.0, 28.0);
    PASSING_FLYWHEELMAP.put(4.0, 26.0);

    PASSING_TOFMAP.put(4.0, 1.25);
    PASSING_TOFMAP.put(5.0, 1.27);
    PASSING_TOFMAP.put(6.0, 1.28);

    // TOF Data (needs to be retested)
    timeOfFlightMap.put(1.5, 0.95);
    timeOfFlightMap.put(2.0, 0.90);
    timeOfFlightMap.put(2.6, 0.94);
    timeOfFlightMap.put(3.0, 1.03);
    timeOfFlightMap.put(3.5, 0.97);
    timeOfFlightMap.put(4.0, 1.07);
  }

  private static Rotation2d hoodRegression(double x) {
    return Rotation2d.fromDegrees(4.0 * x - 2.0);
  }

  private static AngularVelocity shotRegression(double x) {
    return RotationsPerSecond.of(2.27 * x + 22.3);
  }

  private static double tofRegression(double x) {
    return 0.076 * x + 0.982;
  }

  public static double getTOF(double distance, boolean passing) {
    return passing ? PASSING_TOFMAP.get(distance) : tofRegression(distance);
  }

  public static Pair<Angle, AngularVelocity> getShotParameters(double distance, boolean passing) {
    if (passing) {
      return Pair.of(
          PASSING_HOODMAP.get(distance).getMeasure(),
          RotationsPerSecond.of(PASSING_FLYWHEELMAP.get(distance)));
    } else {
      return Pair.of(hoodRegression(distance).getMeasure(), shotRegression(distance));
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
    } else if (closestTarget < TurretConstants.minAzimuthAngle.in(Rotations)) {
      closestTarget += 1;
    }
    return Rotations.of(closestTarget);
  }

  public static Translation2d getPassingTarget() {
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
    double distance = turretPose.getTranslation().getDistance(target);
    Logger.recordOutput("TurretCalculator/targetDistance", distance);
    var az = calculateAzimuthAngle(azimuth.getMeasure(), currentAzimuthAngle);
    var hoodShot = getShotParameters(distance, passing);
    return new TurretParameters(
        az, RotationsPerSecond.of(0.0), hoodShot.getFirst(), hoodShot.getSecond());
  }

  // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
  public static TurretParameters turretIterativeMovingSetpoint(
      Translation2d target, boolean passing, Angle currentAzimuthAngle) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    robotPose =
        robotPose.exp(RobotState.getInstance().getFieldVelocity().toTwist2d(0.003)); // phase shift
    Pose2d turretPose = new Pose3d(robotPose).transformBy(TurretConstants.robotToTurret).toPose2d();
    Translation2d fieldVelocity =
        rigidPointVelocity(
                RobotState.getInstance().getRobotVelocity(),
                TurretConstants.robotToTurret.getTranslation().toTranslation2d())
            .rotateBy(robotPose.getRotation());
    if (fieldVelocity.getNorm() < 0.25)
      return getStationarySetpoint(target, passing, currentAzimuthAngle);

    double distance = turretPose.getTranslation().getDistance(target);
    Logger.recordOutput("TurretCalculator/targetDistance", distance);
    double tof = getTOF(distance, passing);
    for (int i = 0; i < kMaxIterations; ++i) {
      Translation2d futureTurretPos = turretPose.getTranslation().plus(fieldVelocity.times(tof));
      double lastDist = distance;
      distance = futureTurretPos.getDistance(target);
      if (Math.abs(distance - lastDist) < kConvergenceEpsilon) break;
      tof = getTOF(distance, passing);
    }
    Translation2d aimVector =
        target.minus(turretPose.getTranslation().plus(fieldVelocity.times(tof)));

    Rotation2d azimuthAngle = aimVector.getAngle().minus(robotPose.getRotation());
    double cross2d =
        aimVector.getY() * fieldVelocity.getX() - aimVector.getX() * fieldVelocity.getY();
    double azVelRPS =
        (cross2d / aimVector.getSquaredNorm())
            - RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;

    var az = calculateAzimuthAngle(azimuthAngle.getMeasure(), currentAzimuthAngle);
    var hoodShot = getShotParameters(distance, passing);
    return new TurretParameters(
        az, RotationsPerSecond.of(azVelRPS), hoodShot.getFirst(), hoodShot.getSecond());
  }

  private static Translation2d rigidPointVelocity(ChassisSpeeds speeds, Translation2d r) {
    return new Translation2d(
        speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * r.getY(),
        speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * r.getX());
  }
}
