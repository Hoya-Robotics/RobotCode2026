package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotConfig;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final int index;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private ModuleIOOutputs outputs;

  private SimpleMotorFeedforward ffModel =
      new SimpleMotorFeedforward(RobotConfig.driveKs, RobotConfig.driveKv);

  public Module(int index, ModuleIO io) {
    this.index = index;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Module" + index, inputs);
  }

  public void applyOutputs() {
    io.applyOutputs(outputs);
  }

  public void runSetpoint(SwerveModuleState setpoint) {
    setpoint.optimize(inputs.absoluteSteerHeading);
    setpoint.cosineScale(inputs.absoluteSteerHeading);

    outputs.driveVelocityRadps = setpoint.speedMetersPerSecond / RobotConfig.wheelRadius.in(Meters);
    outputs.driveFeedforward = ffModel.calculate(outputs.driveVelocityRadps);
    outputs.steerHeading = setpoint.angle;
  }
}
