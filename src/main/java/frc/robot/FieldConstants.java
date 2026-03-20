package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.AllianceFlip;
import java.io.IOException;

public class FieldConstants {
  public static AprilTagFieldLayout aprilLayout;

  static {
    try {
      aprilLayout =
          new AprilTagFieldLayout(
              Filesystem.getDeployDirectory().toPath().resolve("apriltags/2026-welded.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Distance fieldLength = Meters.of(aprilLayout.getFieldLength());
  public static final Distance fieldWidth = Meters.of(aprilLayout.getFieldWidth());
  public static final Distance centerLine = fieldLength.div(2.0);
  public static final Distance neutralZoneLength = Inches.of(283);
  public static final Distance neutralZoneStart = centerLine.minus(neutralZoneLength.div(2.0));
  public static final Distance neutralZoneEnd = centerLine.plus(neutralZoneLength.div(2.0));
  public static final Distance bumpWidth = Inches.of(73.0);
  public static final Distance trenchWidth = Inches.of(65.65);
  public static final Distance trenchDepth = Inches.of(47.0);

  public static final Distance passingY = trenchWidth.plus(bumpWidth.div(2.0));
  public static final Distance passingX = Meters.of(3.0);

  public static final Translation3d hubCenter =
      new Translation3d(neutralZoneStart, fieldWidth.div(2.0), Inches.of(72.0));
  public static final Translation2d humanOutpost =
      new Translation2d(
          RobotConfig.bumperWidthX.in(Meters), aprilLayout.getTagPose(29).get().getY());
  public static final Translation2d allianceTrenchShot =
      new Translation2d(
          neutralZoneStart.minus(RobotConfig.bumperWidthX),
          aprilLayout.getTagPose(28).get().getMeasureY());

  public static boolean underTrench(Pose2d pose) {
    pose = AllianceFlip.apply(pose);
    return pose.getMeasureX()
            .lt(neutralZoneStart.plus(trenchDepth.div(2.0).plus(RobotConfig.bumperWidthX.div(2))))
        && pose.getMeasureX()
            .gt(neutralZoneStart.minus(trenchDepth.div(2.0).plus(RobotConfig.bumperWidthX.div(2))))
        && (pose.getMeasureY().lt(trenchWidth)
            || pose.getMeasureY().gt(fieldWidth.minus(trenchWidth)));
  }

  public static boolean inNeutralZone(Pose2d pose) {
    return pose.getMeasureX().gte(neutralZoneStart) && pose.getMeasureX().lte(neutralZoneEnd);
  }

  public static boolean inAllianceZone(Pose2d pose) {
    return pose.getMeasureX().lt(Meters.of(AllianceFlip.applyX(neutralZoneStart.in(Meters))));
  }

  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }
}
