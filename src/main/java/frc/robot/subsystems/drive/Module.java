package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputMode;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOOutputs;
import org.littletonrobotics.junction.Logger;


public class Module {
  private final ModuleIO io;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private ModuleIOOutputs outputs = new ModuleIOOutputs();

  private final Translation2d chassisPosition;
  private final int index;

  private final SimpleMotorFeedforward ffModel =
      new SimpleMotorFeedforward(DriveConstants.drivekS, DriveConstants.drivekV);

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    this.chassisPosition = DriveConstants.modulePositions[index];
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  public void periodicAfter() {
    if (DriverStation.isDisabled()) outputs.mode = ModuleIOOutputMode.BRAKE;
    io.applyOutputs(outputs);
  }

  public void runState(SwerveModuleState state) {
		state.optimize(inputs.absoluteTurnHeading);
		state.cosineScale(inputs.absoluteTurnHeading);

    // w = v / r
    double driveVelocityRps =
        state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters);
    Logger.recordOutput("Module" + index + "/outputVelocityRps", driveVelocityRps);

    outputs.turnHeading = state.angle;
    outputs.driveVelocity = RadiansPerSecond.of(driveVelocityRps);
    outputs.feedforward = ffModel.calculate(driveVelocityRps);
    outputs.mode = ModuleIOOutputMode.DRIVE;
  }

  public void setMode(ModuleIOOutputMode mode) {
    Logger.recordOutput("Module" + index + "/outputMode", mode);
    outputs.mode = mode;
  }

  public Translation2d getChassisPosition() {
    return chassisPosition;
  }

  public Distance getDrivePosition() {
    return inputs.drivePosition;
  }

  public Rotation2d getHeading() {
    return inputs.absoluteTurnHeading;
  }

  public Translation2d getVelocity() {
    // v = w * r
    double vmps = inputs.driveVelocity.in(RadiansPerSecond) * getChassisPosition().getNorm();
    return new Translation2d(vmps, inputs.absoluteTurnHeading);
  }
}
