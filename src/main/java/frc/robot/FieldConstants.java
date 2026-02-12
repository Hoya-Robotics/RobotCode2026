package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.AllianceFlip;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class FieldConstants {
  public static final AprilTagFieldLayout aprilLayout;

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  static {
    try {
      aprilLayout =
          new AprilTagFieldLayout(
              Path.of("src", "main", "deploy", "apriltags", "2026-welded.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static Pair<Pose2d, Pose2d> nearestTrenchWaypoints(Pose2d robotPose) {
    boolean rightSide = robotPose.getY() < fieldWidth / 2.0;
    boolean neutralStart = inNeutralZone(robotPose);

    var allianceEntrance = rightTrenchAllianceZoneEntrance;
    var neutralEntrance = rightTrenchNeutralZoneEntrance;
    if (!rightSide) {
      allianceEntrance =
          new Translation2d(allianceEntrance.getX(), fieldWidth - allianceEntrance.getY());
      neutralEntrance =
          new Translation2d(neutralEntrance.getX(), fieldWidth - neutralEntrance.getY());
    }

    Rotation2d heading = neutralStart ? Rotation2d.k180deg : Rotation2d.kZero;
    var translations =
        neutralStart
            ? List.of(neutralEntrance, allianceEntrance)
            : List.of(allianceEntrance, neutralEntrance);
    var path =
        translations.stream().map(AllianceFlip::apply).map(t -> new Pose2d(t, heading)).toList();
    return Pair.of(path.get(0), path.get(1));
  }

  public static boolean inNeutralZone(Pose2d robotPose) {
    Distance robotX = robotPose.getMeasureX();
    return robotX.lt(startingLineLengthX.plus(neutralZoneLengthX))
        && robotX.gt(startingLineLengthX);
  }

  public static final double fieldLength = aprilLayout.getFieldLength();
  public static final double fieldWidth = aprilLayout.getFieldWidth();

  // https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
  public static final Distance startingLineLengthX = Units.Inches.of(158.6);
  public static final Distance neutralZoneLengthX = Units.Inches.of(283);

  public static final Distance trenchLengthX = Units.Inches.of(65.65);
  public static final Distance trenchWidthY = Units.Inches.of(47.0);

  public static final Translation2d rightTrenchAllianceZoneEntrance =
      new Translation2d(startingLineLengthX, trenchWidthY.div(2.0));
  public static final Translation2d rightTrenchNeutralZoneEntrance =
      new Translation2d(startingLineLengthX.plus(trenchLengthX), trenchWidthY.div(2.0));
}
