package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.util.LimelightHelpers;
import java.util.Arrays;
import java.util.Objects;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  public record FiducialObservation(
      int id, double ambiguity, double txnc, double tync, double area) {
    public static FiducialObservation[] arrayFromLL(LimelightHelpers.RawFiducial[] fiducials) {
      if (fiducials == null) {
        return new FiducialObservation[0];
      }
      return Arrays.stream(fiducials)
          .map(f -> new FiducialObservation(f.id, f.ambiguity, f.txnc, f.tync, f.ta))
          .filter(Objects::nonNull)
          .toArray(FiducialObservation[]::new);
    }
  }

  public record MultitagEstimate(
      Pose2d pose,
      double avgTagArea,
      double latency,
      double timestamp,
      double quality,
      int tagCount) {
    public static MultitagEstimate fromLL(LimelightHelpers.PoseEstimate estimate) {
      return new MultitagEstimate(
          estimate.pose,
          estimate.avgTagArea,
          estimate.latency,
          estimate.timestampSeconds,
          estimate.tagCount > 1 ? 1.0 : 1.0 - estimate.rawFiducials[0].ambiguity,
          estimate.tagCount);
    }
  }

  @AutoLog
  public class VisionIOInputs {
    public boolean seesTarget;
    public Pose3d pose3d;
    public MultitagEstimate mt1;
    public MultitagEstimate mt2;
    public FiducialObservation[] fiducials;
    public double[] stddevs = new double[12];
  }

  public default CameraConfig getConfig() {
    return new CameraConfig("null", Transform3d.kZero);
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setRobotToCamera(Transform3d robotToCamera) {}

  public default void setRobotOrientation(Rotation2d gyroYaw) {}

  public default void captureRewind(double duration) {}
}
