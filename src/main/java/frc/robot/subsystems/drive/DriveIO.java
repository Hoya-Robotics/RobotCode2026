package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs extends SwerveDriveState {
    public Rotation2d gyroYaw;

    public DriveIOInputs() {
      this.Pose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    }

    public void fromDriveState(SwerveDriveState state) {
      this.Pose = state.Pose;
      this.gyroYaw = state.Pose.getRotation();
      this.SuccessfulDaqs = state.SuccessfulDaqs;
      this.FailedDaqs = state.FailedDaqs;
      this.ModuleStates = state.ModuleStates;
      this.ModuleTargets = state.ModuleTargets;
      this.Speeds = state.Speeds;
      this.OdometryPeriod = state.OdometryPeriod;
    }
  }

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void applyRequest(SwerveRequest request) {}

  public default void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> stdDevs) {}

  public default void resetOdometry(Pose2d override) {}
}
