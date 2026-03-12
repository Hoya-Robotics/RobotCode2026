package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.AllianceFlip;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class FieldConstants {
  public static AprilTagFieldLayout aprilLayout;

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  static {
    try {
      aprilLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath().resolve("apriltags/2026-welded.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static boolean inNeutralZone(Pose2d robotPose) {
    double center = fieldLength / 2.0;
    double neutralZoneNear =
        center - Units.inchesToMeters(120) + RobotConfig.bumperWidthX.in(Meters);
    double neutralZoneFar =
        center + Units.inchesToMeters(120) - RobotConfig.bumperWidthX.in(Meters);
    double robotX = robotPose.getX();
    return robotX < neutralZoneFar && robotX > neutralZoneNear;
  }

  public static final double fieldLength = aprilLayout.getFieldLength();
  public static final double fieldWidth = aprilLayout.getFieldWidth();

  // https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
  public static final Distance startingLineLengthX = aprilLayout.getTagPose(26).get().getMeasureX();

  public static final Distance trenchLengthX = Inches.of(65.65);
  public static final Distance trenchWidthY = Inches.of(47.0);
  public static final Distance trenchCenter = trenchWidthY.div(2.0);

  public static final Pose2d humanStation = new Pose2d(0.5464, 0.6621, Rotation2d.k180deg);

  public static final Pose2d getHumanStation() {
    return AllianceFlip.apply(humanStation);
  }

  public static class Hub {
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    public static final Translation3d topCenterPoint =
        new Translation3d(
            aprilLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            aprilLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, innerHeight);

    public static Translation3d getTopCenter() {
      return AllianceFlip.apply(topCenterPoint);
    }

    public static Translation3d getInnerCenter() {
      return AllianceFlip.apply(innerCenterPoint);
    }
  }

  public static class Trench {
    public static final Distance trenchWidthX = Inches.of(47.0);
    public static final Translation2d rightTrenchCenter =
        new Translation2d(startingLineLengthX, trenchCenter);
    public static final Translation2d rightTrenchClose =
        new Translation2d(startingLineLengthX.minus(RobotConfig.bumperWidthX), trenchCenter);
    public static final Translation2d rightTrenchFar =
        new Translation2d(
            startingLineLengthX.plus(trenchWidthX).plus(RobotConfig.bumperWidthX), trenchCenter);

    public static Optional<Pose2d> triggerTrenchAlign() {
      return Optional.empty();
    }

    private static List<Pose2d> resolve(Pose2d pose, Translation2d... ps) {
      Rotation2d targetHeading = inNeutralZone(pose) ? Rotation2d.k180deg : Rotation2d.kZero;
      return Arrays.stream(ps).map(p -> AllianceFlip.apply(new Pose2d(p, targetHeading))).toList();
    }

    private static Translation2d flipPoint(Translation2d p) {
      return new Translation2d(p.getX(), fieldWidth - p.getY());
    }

    public static Pose2d getCloseTrench(Pose2d pose) {
      return pose.nearest(resolve(pose, rightTrenchClose, flipPoint(rightTrenchClose)));
    }

    public static Pose2d getFarTrench(Pose2d pose) {
      return pose.nearest(resolve(pose, rightTrenchFar, flipPoint(rightTrenchFar)));
    }
  }
}
