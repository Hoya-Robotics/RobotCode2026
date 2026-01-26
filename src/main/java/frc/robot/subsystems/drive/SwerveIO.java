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
    double driveStatorCurrent;
    double driveSupplyCurrent;
    double driveVoltsApplied;

    double steerStatorCurrent;
    double steerSupplyCurrent;
    double steerVoltsApplied;
  }

  default void setTelemetryInputs(SwerveIOInputs inputs) {}

  default void applyRequest(SwerveRequest request) {}

  default void updateModuleInputs(ModuleIOInputs[] inputs) {}
}
