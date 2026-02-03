package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState.*;
import java.util.Optional;

public class Vision extends SubsystemBase {

  public class LimelightOutputs {
    public Optional<Boolean> setRewind;
    public Optional<Integer> captureRewindWithDuration;
    public Optional<Double[]> imuSeedData;
  }

  private final LocalizationCameraIO[] localizers;
  private LocalizationInputsAutoLogged[] localizerInputs;

  public Vision(LocalizationCameraIO[] localizers) {
    this.localizers = localizers;
    localizerInputs = new LocalizationInputsAutoLogged[localizers.length];
  }

	@Override
	public void periodic() {
	}
}
