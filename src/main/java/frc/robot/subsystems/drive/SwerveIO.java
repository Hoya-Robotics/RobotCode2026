package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveIOInputs {
    Pose2d pose = Pose2d.kZero;
    SwerveModulePosition[] modulePositions;
    Rotation2d yaw = Rotation2d.kZero;
    ChassisSpeeds speeds;
    double timestamp = 0.0;
  }

  @AutoLog
  public static class ModuleIOInputs {
    double driveStatorCurrent = 0.0;
    double driveSupplyCurrent = 0.0;
    double driveVoltsApplied = 0.0;

    double steerStatorCurrent = 0.0;
    double steerSupplyCurrent = 0.0;
    double steerVoltsApplied = 0.0;
  }

  default void setTelemetryInputs(SwerveIOInputs inputs) {}

  default void applyRequest(SwerveRequest request) {}

  default void updateSim() {}

  default Pose2d getSimPose() {
    return Pose2d.kZero;
  }

  default void updateModuleInputs(ModuleIOInputs[] inputs) {}
}
