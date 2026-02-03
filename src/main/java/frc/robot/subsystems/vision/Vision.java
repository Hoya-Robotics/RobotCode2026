package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState.*;

public class Vision extends SubsystemBase {

	public class LimelightOutputs {
		public Optional<Boolean> setRewind;
		public Optional<Integer> captureRewindWithDuration;
		public Optional<Double[]> imuSeedData;
	}

	private List<LocalizationCameraIO> localizers;

  public Vision() {
	}
}
