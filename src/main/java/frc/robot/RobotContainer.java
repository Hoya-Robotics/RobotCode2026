// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.*;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  public final Drive drive;
  public final XboxController driveController = new XboxController(0);
  public final DoubleSupplier driveX = () -> -driveController.getLeftY();
  public final DoubleSupplier driveY = () -> driveController.getLeftX();
  public final DoubleSupplier turnX = () -> -driveController.getRightY();
  public SwerveDriveSimulation swerveSim = null;

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driveController);
        break;
      case SIM:
        swerveSim =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, Rotation2d.kZero));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);
        RobotState.getInstance().addSimulatedPoseCallback(swerveSim::getSimulatedDriveTrainPose);
        var simMods = swerveSim.getModules();
        drive =
            new Drive(
                new GyroIOSim(swerveSim.getGyroSimulation()),
                new ModuleIOGeneralSim(simMods[0]),
                new ModuleIOGeneralSim(simMods[1]),
                new ModuleIOGeneralSim(simMods[2]),
                new ModuleIOGeneralSim(simMods[3]),
                driveController);
        break;
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driveController);
        break;
    }

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void displaySimField() {
    Logger.recordOutput("FieldSimulation/robotPose", swerveSim.getSimulatedDriveTrainPose());
  }
}
