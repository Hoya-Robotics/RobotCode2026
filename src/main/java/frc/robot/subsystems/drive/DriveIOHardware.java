package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotState.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

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
          /*
          RobotState.getInstance()
              .addOdometryObservation(
                  new OdometryObservation(
                      swerveDriveState.Speeds,
                      swerveDriveState.ModulePositions,
                      swerveDriveState.Pose.getRotation(),
                      swerveDriveState.Timestamp));*/
        };
    registerTelemetry(telemetryConsumer);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (telemetryCache.get() == null) return;
    inputs.fromDriveState(telemetryCache.get());
  }

  @Override
  public void applyRequest(SwerveRequest request) {
    super.setControl(request);
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> stdDevs) {
    super.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  @Override
  public void resetOdometry(Pose2d override) {
    super.resetPose(override);
  }
}
