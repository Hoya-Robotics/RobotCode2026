package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotConfig;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation simModule;

  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController steerMotor;

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

  private double driveFFVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation simModule) {
    this.simModule = simModule;
    this.driveMotor =
        this.simModule.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(100));
    this.steerMotor = this.simModule.useGenericControllerForSteer().withCurrentLimit(Amps.of(100));

    steerClosedLoop.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotor.requestVoltage(
        Volts.of(
            driveFFVolts
                + driveClosedLoop.calculate(
                    simModule.getDriveWheelFinalSpeed().in(RadiansPerSecond))));
    steerMotor.requestVoltage(
        Volts.of(steerClosedLoop.calculate(simModule.getSteerAbsoluteFacing().getRadians())));

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
    driveFFVolts = outputs.driveFeedforward;

    steerClosedLoop.setSetpoint(outputs.steerHeading.getRadians());
    driveClosedLoop.setSetpoint(outputs.driveVelocityRadps);
  }
}
