package frc.robot.subsystems.vision;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    boolean connected;
    AprilFiducial bestTarget;
    AprilFiducial[] allTargets;
    Angle megatagYaw;
  }

  public static record AprilFiducial(
      int camera, double tx, double ty, int tid, double timestamp, double ambiguity) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
