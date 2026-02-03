package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConfig;
import frc.robot.RobotConfig.CameraConfig;

public class LimelightLocalizationCamera implements LocalizationCameraIO {
	private final DoubleArraySubscriber cameraPoseTargetspaceSubscriber;
	private final DoubleArraySubscriber megatag2Subscriber;
	private final DoubleSubscriber clSubscriber;
	private final DoubleSubscriber tlSubscriber;
	private final IntegerSubscriber tidSubscriber;

	private final DoubleArrayPublisher rewindCapturePublisher;
	private final DoublePublisher rewindEnablePublisher;
	private final IntegerPublisher throttlePublisher;
	private final IntegerPublisher imuModePublisher;
	private final DoubleArrayPublisher imuSeedDataPublisher;

	private boolean disabledState = false;
	private double captures = 0.0;

	public LimelightLocalizationCamera(CameraConfig config) {
		var table = NetworkTableInstance.getDefault().getTable(config.name());

		megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

		cameraPoseTargetspaceSubscriber = table.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[] {});
		tlSubscriber = table.getDoubleTopic("tl").subscribe(0.0);
		clSubscriber = table.getDoubleTopic("cl").subscribe(0.0);
		tidSubscriber = table.getIntegerTopic("tid").subscribe(0);

		throttlePublisher = table.getIntegerTopic("throttle_set").publish();
		imuModePublisher = table.getIntegerTopic("imumode_set").publish();
		imuSeedDataPublisher = table.getDoubleArrayTopic("imu").publish();
		rewindEnablePublisher = table.getDoubleTopic("rewind_enable_set").publish();
		rewindCapturePublisher = table.getDoubleArrayTopic("capture_rewind").publish();
	}

	@Override
	public void updateInputs(LocalizationInputs inputs) {
		NetworkTableInstance.getDefault().flush();

		double latency = tlSubscriber.get() + clSubscriber.get();
		inputs.timestamp = Timer.getFPGATimestamp() - latency;

		if (DriverStation.isDisabled()) {
			disabledState = true;
			throttlePublisher.set(200);
			imuModePublisher.set(1);
		} else if (disabledState) {
			disabledState = false;
			throttlePublisher.set(0);
			imuModePublisher.set(RobotConfig.matchImuMode);
		}

		int tid = (int) tidSubscriber.get();

		// Check for hub tags
		if (tid == 9 || tid == 10 || tid == 25 || tid == 26) {
			double[] cameraposeTargetspace = cameraPoseTargetspaceSubscriber.get();
			inputs.cameraToHub = Optional.of(new Pose3d(
				cameraposeTargetspace[0],
				cameraposeTargetspace[1],
				cameraposeTargetspace[2],
				new Rotation3d(
					Units.degreesToRadians(cameraposeTargetspace[3]),
					Units.degreesToRadians(cameraposeTargetspace[5]),
					Units.degreesToRadians(cameraposeTargetspace[4])
				))
			);
		}

		// Read all updated megatag2 estimates
		List<MT2Observation> observations = new ArrayList<>();
		for (var megatagSample : megatag2Subscriber.readQueue()) {
			Pose3d pose = deserializeLLPose(megatagSample.value);
			MT2Observation observation = new MT2Observation(
				megatagSample.timestamp - megatagSample.value[6],
				pose,
				megatagSample.value[9],
				(int) megatagSample.value[7]
			);
			observations.add(observation);
		}
		inputs.globalPoseObservations = observations.toArray(MT2Observation[]::new);
	}

	@Override
	public void applyOutputs(LocalizationOutputs outputs) {
		outputs.llOutputs.setRewind.ifPresent((enable) -> {
			double value = enable ? 1.0 : 0.0;
			rewindEnablePublisher.set(value);
		});

		outputs.llOutputs.captureRewindWithDuration.ifPresent((duration) -> {
			rewindCapturePublisher.set(new double[] { captures, duration });
			captures += 1;
		});

		outputs.llOutputs.imuSeedData.ifPresent((dataArray) -> {
			double[] data = Arrays.stream(dataArray)
				.mapToDouble(Double::doubleValue)
				.toArray();
			imuSeedDataPublisher.set(data);
		});
	}

	private static Pose3d deserializeLLPose(double[] values) {
		return new Pose3d(
			values[0],
			values[1],
			values[2],
			new Rotation3d(
					Units.degreesToRadians(values[3]),
					Units.degreesToRadians(values[4]),
					Units.degreesToRadians(values[5])
			)
		);
	}
}
