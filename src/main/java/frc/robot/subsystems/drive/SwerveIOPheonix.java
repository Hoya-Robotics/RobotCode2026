package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SwerveIOPheonix extends SwerveDrivetrain implements SwerveIO {
  private List<HashMap<String, BaseStatusSignal>> moduleSignals = new ArrayList<>();

  public SwerveIOPheonix(
      SwerveDrivetrainConstants constants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);

    for (int i = 0; i < 4; ++i) {
      var signalMap = new HashMap<String, BaseStatusSignal>();

      var driveMotor = this.getModule(i).getDriveMotor();
      var steerMotor = this.getModule(i).getSteerMotor();

      signalMap.put("driveStatorCurrent", driveMotor.getStatorCurrent());
      signalMap.put("driveSupplyCurrent", driveMotor.getSupplyCurrent());
      signalMap.put("driveVoltsApplied", driveMotor.getMotorVoltage());

      signalMap.put("steerStatorCurrent", steerMotor.getStatorCurrent());
      signalMap.put("steerSupplyCurrent", steerMotor.getSupplyCurrent());
      signalMap.put("steerVoltsApplied", steerMotor.getMotorVoltage());

      moduleSignals.add(signalMap);
    }
  }

  @SuppressWarnings("unchecked")
  @Override
  public void setTelemetryInputs(SwerveIOInputs inputs) {
    this.registerTelemetry(
        rawState -> {
          SwerveDriveState state = (SwerveDriveState) rawState;
          inputs.pose = state.Pose;
          inputs.speeds = state.Speeds;
          inputs.yaw = state.RawHeading;
          inputs.modulePositions = state.ModulePositions;
          inputs.timestamp = state.Timestamp;
        });
  }

  @Override
  public void updateModuleInputs(ModuleIOInputs[] inputs) {
    for (int i = 0; i < 4; ++i) {
      inputs[i].driveStatorCurrent =
          moduleSignals.get(i).get("driveStatorCurrent").getValueAsDouble();
      inputs[i].driveSupplyCurrent =
          moduleSignals.get(i).get("driveSupplyCurrent").getValueAsDouble();
      inputs[i].driveVoltsApplied =
          moduleSignals.get(i).get("driveVoltsApplied").getValueAsDouble();

      inputs[i].steerStatorCurrent =
          moduleSignals.get(i).get("steerStatorCurrent").getValueAsDouble();
      inputs[i].steerSupplyCurrent =
          moduleSignals.get(i).get("steerSupplyCurrent").getValueAsDouble();
      inputs[i].steerVoltsApplied =
          moduleSignals.get(i).get("steerVoltsApplied").getValueAsDouble();
    }
  }

  @Override
  public void applyRequest(SwerveRequest request) {
    this.setControl(request);
  }
}
