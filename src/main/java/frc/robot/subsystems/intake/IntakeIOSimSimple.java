// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeIOSimSimple implements IntakeIO {
  private DCMotor gearbox = DCMotor.getFalcon500(1);
  private ElevatorSim intakeSim =
      new ElevatorSim(
          gearbox, 10.3846, 0.1, Inches.of(0.9).in(Meters), 0, Inches.of(11).in(Meters), false, 0);

  private PIDController intakePID = new PIDController(100, 0, 0);
  private double voltageApplied = 0.0;

  public IntakeIOSimSimple() {
    intakeSim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.setInputVoltage(voltageApplied);
    intakeSim.update(0.020);

    inputs.extendConnected = true;
    // Position is in inches due to SensorToMechanismRatio converting rotations to inches
    inputs.extendPosition = Meters.of(intakeSim.getPositionMeters());
    inputs.extendVelocity = MetersPerSecond.of(intakeSim.getVelocityMetersPerSecond());
    inputs.extendVoltageApplied = Volts.of(voltageApplied);
    inputs.extendCurrent = Amps.of(0.0);

    inputs.intakeConnected = false;
    inputs.intakeVoltageApplied = Volts.zero();
    inputs.intakeCurrent = Amps.zero();
    inputs.intakeVelocity = RPM.zero();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    Logger.recordOutput("Intake/extensionSetpoint", outputs.extendSetpointInches);
    Logger.recordOutput("Intake/intakeSetpoint", outputs.intakeVelocityRPM);

    intakePID.setSetpoint(Units.inchesToMeters(outputs.extendSetpointInches));

    voltageApplied =
        switch (outputs.extendControlType) {
          case POSITION -> intakePID.calculate(intakeSim.getPositionMeters());
          case VOLTAGE -> outputs.extendVoltage;
        };
  }
}
