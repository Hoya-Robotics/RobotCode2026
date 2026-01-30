package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.IOException;
import java.nio.file.Path;

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

  public static final double fieldLength = aprilLayout.getFieldLength();
  public static final double fieldWidth = aprilLayout.getFieldWidth();

  /** Hub related constants */
  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d topCenterPoint =
        new Translation3d(
            aprilLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            aprilLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, innerHeight);

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            aprilLayout.getTagPose(4).get().getX() + width / 2.0, fieldWidth / 2.0, height);
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // Hub faces
    public static final Pose2d nearFace = aprilLayout.getTagPose(26).get().toPose2d();
    public static final Pose2d farFace = aprilLayout.getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace = aprilLayout.getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace = aprilLayout.getTagPose(21).get().toPose2d();
  }
}
