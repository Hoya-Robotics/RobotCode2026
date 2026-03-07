package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotConfig.CameraConfig;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public class VisionIOInputs {
    public boolean isConnected = false;

    public Pose2d poseEstimate = Pose2d.kZero;
    public Matrix<N3, N1> stdDevs;
    public double avgTagDist = 0.0;
    public double timestamp = 0.0;
    public int numTags = 0;

    public boolean hubInSight = false;
    public int primaryTid = 0;
    public Pose3d robotToHub = Pose3d.kZero;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default CameraConfig getConfig() {
    return new CameraConfig("foo", Transform3d.kZero);
  }

  public default void captureRewind(double duration) {}
}
