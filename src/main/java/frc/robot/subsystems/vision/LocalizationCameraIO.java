package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.Vision.LimelightOutputs;

public interface LocalizationCameraIO {
	@AutoLog
	public static class LocalizationInputs {
		public boolean isConnected = false;
		public double timestamp = 0.0;
		public Optional<Pose3d> cameraToHub = Optional.empty();
		public MT2Observation[] globalPoseObservations;
	}

	public record MT2Observation(
		double timestamp,
		Pose3d pose,
		double avgTagDistance,
		int tagCount
	) {}

	public static class LocalizationOutputs {
		public LimelightOutputs llOutputs;
	}

	default void updateInputs(LocalizationInputs input) {}
	default void applyOutputs(LocalizationOutputs outputs) {}
}
