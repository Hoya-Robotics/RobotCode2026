package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotConfig;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation simModule;

  private final GenericMotorController driveMotor;
  private final GenericMotorController steerMotor;

  private final PIDController driveClosedLoop =
      new PIDController(
          RobotConfig.simDriveMotorGains.kp(),
          RobotConfig.simDriveMotorGains.ki(),
          RobotConfig.simDriveMotorGains.kd());
  private final PIDController steerClosedLoop =
      new PIDController(
          RobotConfig.simSteerMotorGains.kp(),
          RobotConfig.simSteerMotorGains.ki(),
          RobotConfig.simSteerMotorGains.kd());

  public ModuleIOSim(SwerveModuleSimulation simModule) {
    this.simModule = simModule;
    this.driveMotor = simModule.useGenericControllerForSteer();
    this.steerMotor = simModule.useGenericControllerForSteer();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.drivePositionRads = simModule.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadps = simModule.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveStatorCurrentAmps = simModule.getDriveMotorStatorCurrent().in(Amps);
    inputs.driveSupplyCurrentAmps = simModule.getDriveMotorSupplyCurrent().in(Amps);
    inputs.driveVoltsApplied = driveMotor.getAppliedVoltage().in(Volts);

    inputs.steerConnected = true;
    inputs.absoluteSteerHeading = simModule.getSteerAbsoluteFacing();
    inputs.steerVelocityRadps = simModule.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.steerStatorCurrentAmps = simModule.getSteerMotorStatorCurrent().in(Amps);
    inputs.steerSupplyCurrentAmps = simModule.getSteerMotorSupplyCurrent().in(Amps);
    inputs.steerVoltsApplied = steerMotor.getAppliedVoltage().in(Volts);
  }

  @Override
  public void applyOutputs(ModuleIOOutputs outputs) {
    double driveVoltage =
        outputs.driveFeedforward + driveClosedLoop.calculate(outputs.driveVelocityRadps);
    double steerVoltage = steerClosedLoop.calculate(outputs.steerHeading.getRadians());

    driveMotor.requestVoltage(Volts.of(driveVoltage));
    steerMotor.requestVoltage(Volts.of(steerVoltage));
  }
}
