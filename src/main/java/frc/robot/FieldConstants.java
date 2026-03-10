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
import java.util.List;

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
    Distance robotX = robotPose.getMeasureX();
    return robotX.lt(startingLineLengthX.plus(neutralZoneLengthX))
        && robotX.gt(startingLineLengthX);
  }

  public static final double fieldLength = aprilLayout.getFieldLength();
  public static final double fieldWidth = aprilLayout.getFieldWidth();

  // https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
  public static final Distance startingLineLengthX = Inches.of(158.6);
  public static final Distance neutralZoneLengthX = Inches.of(283);

  public static final Distance trenchLengthX = Inches.of(65.65);
  public static final Distance trenchWidthY = Inches.of(47.0);
  public static final Distance trenchCenter = trenchWidthY.div(2.0);

  public static final Translation2d humanStation = new Translation2d(0.5464, 0.6621);

  public static final Translation2d getHumanStation() {
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
    public static final Translation2d rightTrench =
        new Translation2d(startingLineLengthX, trenchCenter);
    public static final Translation2d leftTrench =
        new Translation2d(startingLineLengthX, Meters.of(fieldWidth).minus(trenchCenter));

    public static Translation2d getRightTrench() {
      return AllianceFlip.apply(rightTrench);
    }

    public static Translation2d getLeftTrench() {
      return AllianceFlip.apply(leftTrench);
    }

    public static Pose2d getNearestTrench(Pose2d pose) {
      return pose.nearest(
          List.of(
              new Pose2d(getRightTrench(), Rotation2d.kZero),
              new Pose2d(getLeftTrench(), Rotation2d.kZero)));
    }
  }
}
