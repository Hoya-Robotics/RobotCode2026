package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.VisionConstants;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.simulation.SimCameraProperties;

public interface VisionIO {
  public record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double avgTagDist,
      double avgTagArea) {
    public boolean isInvalid() {
      /*
         1. Not too high
      2. Not single tag ambiguous
         3. Estimate in field
         4. Not far avg dist
         */
      return pose.getZ() > VisionConstants.zThreshold
          || tagCount == 0
          || (tagCount == 1 && ambiguity > VisionConstants.maxAmbiguity)
          || Math.abs(pose.getX()) < 1e-6
          || Math.abs(pose.getY()) < 1e-6
          || Math.abs(pose.getZ()) < 1e-6
          || pose.getX() < 0.0
          || pose.getY() < 0.0
          || pose.getMeasureX().gt(FieldConstants.fieldLength)
          || pose.getMeasureY().gt(FieldConstants.fieldWidth)
          || avgTagDist > VisionConstants.maxAvgDist;
    }
  }

  @AutoLog
  public class VisionIOInputs {
    public boolean seesTarget = false;
    public PoseObservation[] observations;
    public double[] stddevs = new double[2];
  }

  public default CameraConfig getConfig() {
    return new CameraConfig("null", Transform3d.kZero, new SimCameraProperties());
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void captureRewind(double duration) {}
}
