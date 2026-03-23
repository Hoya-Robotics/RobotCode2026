package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.util.MotorState;

public class TurretIOSim implements TurretIO {
  private DCMotor azimuthGearbox = DCMotor.getKrakenX60(1);
  private DCMotorSim azimuthSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              azimuthGearbox, 0.07867078, TurretConstants.azimuthGearRatio),
          azimuthGearbox);
  private PIDController azimuthController = new PIDController(200, 0, 0);

  private DCMotor hoodGearbox = DCMotor.getKrakenX44(1);
  private DCMotorSim hoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              hoodGearbox, 0.07867078, TurretConstants.hoodGearRatio),
          hoodGearbox);
  private PIDController hoodController = new PIDController(200, 0, 0);

  private AngularVelocity shooterVelocity = RotationsPerSecond.zero();

  public TurretIOSim() {
    azimuthSim.setState(0.0, 0.0);
    hoodSim.setState(0.0, 0.0);
    azimuthController.enableContinuousInput(-0.5, 0.5);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double azimuthVoltageApplied =
        MathUtil.clamp(
            azimuthController.calculate(azimuthSim.getAngularPositionRotations()), -12.0, 12.0);
    double hoodVoltageApplied =
        MathUtil.clamp(
            hoodController.calculate(hoodSim.getAngularPositionRotations()), -12.0, 12.0);

    hoodSim.setInputVoltage(hoodVoltageApplied);
    azimuthSim.setInputVoltage(azimuthVoltageApplied);
    hoodSim.update(1.0 / 50.0);
    azimuthSim.update(1.0 / 50.0);

    inputs.hoodState =
        new MotorState(
            true,
            hoodVoltageApplied,
            0.0,
            hoodSim.getAngularPositionRotations(),
            hoodSim.getAngularVelocity().in(RotationsPerSecond),
            0.0);

    inputs.azimuthState =
        new MotorState(
            true,
            azimuthVoltageApplied,
            0.0,
            azimuthSim.getAngularPositionRotations(),
            azimuthSim.getAngularVelocity().in(RotationsPerSecond),
            0.0);

    inputs.shooterState =
        new MotorState(true, 0.0, 0.0, 0.0, shooterVelocity.in(RotationsPerSecond), 0.0);
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    hoodController.setSetpoint(outputs.hoodSetpoint.in(Rotations));
    azimuthController.setSetpoint(outputs.azimuthSetpoint.in(Rotations));
    shooterVelocity = outputs.shooterSetpoint;
  }
}
