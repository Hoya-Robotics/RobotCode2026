package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final String[] cameraNames = new String[] {"foo"};
  public static final Transform3d[] robotToCameras = new Transform3d[] {Transform3d.kZero};

  public static final Transform2d[] cameraToRobot2d = new Transform2d[robotToCameras.length];

  static {
    for (int i = 0; i < robotToCameras.length; ++i) {
      var inv = robotToCameras[i].inverse();
      cameraToRobot2d[i] =
          new Transform2d(inv.getTranslation().toTranslation2d(), inv.getRotation().toRotation2d());
    }
  }
}
