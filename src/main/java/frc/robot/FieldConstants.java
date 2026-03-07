package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.AllianceFlip;
import java.io.IOException;
import java.nio.file.Path;
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
              Path.of("src", "main", "deploy", "apriltags", "2026-welded.json"));
    } catch (IOException e) {
      // throw new RuntimeException(e);
    }
  }

  public static boolean inNeutralZone(Pose2d robotPose) {
    Distance robotX = robotPose.getMeasureX();
    return robotX.lt(startingLineLengthX.plus(neutralZoneLengthX))
        && robotX.gt(startingLineLengthX);
  }

  // public static final double fieldLength = aprilLayout.getFieldLength();
  public static final double fieldLength = 0.0;
  // public static final double fieldWidth = aprilLayout.getFieldWidth();
  public static final double fieldWidth = 0.0;

  // https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
  public static final Distance startingLineLengthX = Units.Inches.of(158.6);
  public static final Distance neutralZoneLengthX = Units.Inches.of(283);

  public static final Distance trenchLengthX = Units.Inches.of(65.65);
  public static final Distance trenchWidthY = Units.Inches.of(47.0);
  public static final Distance trenchCenter = trenchWidthY.div(2.0);

  // Hub center points calculated from April tag positions
  // Blue hub: tags 9 & 10 at x≈12.519m, y≈3.679m and y≈4.035m
  // Red hub: tags 25 & 26 at x≈4.022m, y≈4.390m and y≈4.035m
  private static final Pose2d blueHubCenter =
      new Pose2d(12.519, (3.679 + 4.035) / 2.0, Rotation2d.kZero);

  public static Pose2d getHubCenter() {
    return AllianceFlip.apply(blueHubCenter);
  }

  @Deprecated // Use getHubCenter() for alliance-aware hub position
  public static final Pose2d hubCenterPoint = blueHubCenter;

  public static final List<Pose2d> trenchPoses;

  static {
    List<Pose2d> blueTrenchPoses =
        List.of(
            new Pose2d(startingLineLengthX, trenchCenter, Rotation2d.kZero),
            new Pose2d(
                startingLineLengthX,
                Units.Meters.of(fieldWidth).minus(trenchCenter),
                Rotation2d.kZero));
    trenchPoses = blueTrenchPoses.stream().map(AllianceFlip::apply).toList();
  }
}
