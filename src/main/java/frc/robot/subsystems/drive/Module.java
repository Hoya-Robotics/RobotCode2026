package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotConfig;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final int index;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private ModuleIOOutputs outputs = new ModuleIOOutputs();

  private SimpleMotorFeedforward ffModel =
      new SimpleMotorFeedforward(RobotConfig.driveKs, RobotConfig.driveKv);

  public Module(int index, ModuleIO io) {
    this.index = index;
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module-" + index, inputs);
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

    Logger.recordOutput(
        "Drive/Module-" + index + "/outputDriveVelocityRadps", outputs.driveVelocityRadps);
    Logger.recordOutput("Drive/Module-" + index + "/driveFeedforward", outputs.driveFeedforward);
    Logger.recordOutput("Drive/Module-" + index + "/steerHeading", outputs.steerHeading);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelocityRadps * RobotConfig.wheelRadius.in(Meters),
        inputs.absoluteSteerHeading);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionRads * RobotConfig.wheelRadius.in(Meters), inputs.absoluteSteerHeading);
  }
}
