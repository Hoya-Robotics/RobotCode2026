package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOGeneralSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSim;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController steerMotor;

  private final PIDController driveController = new PIDController(0.1, 0.0, 0.0);
  private final PIDController steerController = new PIDController(10.0, 0.0, 0.0);

  private double driveFFVolts = 0.0;
  private boolean closedLoop = false;

  public ModuleIOGeneralSim(SwerveModuleSimulation moduleSim) {
    this.moduleSim = moduleSim;

    this.driveMotor = moduleSim.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
    this.steerMotor = moduleSim.useGenericControllerForSteer().withCurrentLimit(Amps.of(60));

    steerController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double driveAppliedVolts = 0.0;
    double steerAppliedVolts = 0.0;
    if (closedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(moduleSim.getDriveWheelFinalSpeed().in(RadiansPerSecond));
      steerAppliedVolts = steerController.calculate(moduleSim.getSteerAbsoluteAngle().in(Radians));
    }

    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    steerMotor.requestVoltage(Volts.of(steerAppliedVolts));

    inputs.driveConnected = true;
    inputs.drivePosition =
        Meters.of(
            moduleSim.getDriveWheelFinalPosition().in(Radians)
                * DriveConstants.wheelRadius.in(Meters));
    inputs.driveVelocity = moduleSim.getDriveWheelFinalSpeed();
    inputs.driveVoltsApplied = driveAppliedVolts;

    inputs.turnConnected = true;
    inputs.absoluteTurnHeading = moduleSim.getSteerAbsoluteFacing();
    inputs.turnVelocity = moduleSim.getSteerAbsoluteEncoderSpeed();
  }

  @Override
  public void applyOutputs(ModuleIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST, BRAKE -> {
        closedLoop = false;
      }
      case DRIVE -> {
        closedLoop = true;
        driveFFVolts = outputs.feedforward;
        driveController.setSetpoint(outputs.driveVelocity.in(RadiansPerSecond));
        steerController.setSetpoint(outputs.turnHeading.getRadians());
      }
    }
  }
}
