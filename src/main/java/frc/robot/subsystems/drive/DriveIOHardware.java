package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import frc.robot.Robot;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class DriveIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DriveIO {
  AtomicReference<SwerveDriveState> telemetryCache = new AtomicReference<>();

  public DriveIOHardware(
      SwerveDrivetrainConstants swerveConstants,
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, swerveConstants, 250.0, moduleConstants);

    this.getOdometryThread().setThreadPriority(99);

    Consumer<SwerveDriveState> telemetryConsumer =
        swerveDriveState -> {
          telemetryCache.set(swerveDriveState.clone());
        };
    registerTelemetry(telemetryConsumer);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (telemetryCache.get() == null) return;
    inputs.fromDriveState(telemetryCache.get());

    double pitch = Math.abs(getPigeon2().getPitch().getValueAsDouble());
    Logger.recordOutput("Drive/pigeonPitchDegrees", pitch);
    if (pitch > 8.0) {
      setStateStdDevs(VecBuilder.fill(9999, 9999, 9999));
    } else {
      setStateStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
    }

    int i = 0;
    for (var module : getModules()) {
      Robot.batteryLogger.reportCurrentUsage(
          "Drive/module" + i + "/drive",
          module.getDriveMotor().getStatorCurrent().getValueAsDouble());
      Robot.batteryLogger.reportCurrentUsage(
          "Drive/module" + i + "/steer",
          module.getSteerMotor().getStatorCurrent().getValueAsDouble());
      i += 1;
    }
  }

  @Override
  public void applyRequest(SwerveRequest request) {
    super.setControl(request);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, Time timestamp, Vector<N3> stdDevs) {
    // Convert FPGA timestamp to CTRE timebase - critical for pose estimator to work correctly
    double ctreTimestamp = com.ctre.phoenix6.Utils.fpgaToCurrentTime(timestamp.in(Seconds));

    // Log vision measurement being passed to CTRE estimator
    Logger.recordOutput("Drive/visionMeasurement/pose", pose);
    Logger.recordOutput("Drive/visionMeasurement/fpgaTimestamp", timestamp.in(Seconds));
    Logger.recordOutput("Drive/visionMeasurement/ctreTimestamp", ctreTimestamp);
    Logger.recordOutput("Drive/visionMeasurement/stdDevX", stdDevs.get(0));
    Logger.recordOutput("Drive/visionMeasurement/stdDevY", stdDevs.get(1));
    Logger.recordOutput("Drive/visionMeasurement/stdDevYaw", stdDevs.get(2));

    super.addVisionMeasurement(pose, ctreTimestamp, stdDevs);
  }

  @Override
  public void resetOdometry(Pose2d override) {
    super.resetPose(override);
  }
}
