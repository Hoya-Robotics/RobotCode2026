// Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or
// share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.util.FuelSim;

public class RobotContainer {
  public final XboxController driveController = new XboxController(0);
  public final Drive drive;
  public FuelSim fuelSim = null;

  // public final Vision vision;

  // public final PlaceholderTurret turret = new PlaceholderTurret();

  public RobotContainer() {
    switch (RobotConfig.getMode()) {
      case SIM:
        drive =
            new Drive(
                driveController,
                new DriveIOSim(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));
        configureFuelSim();
        break;
      default:
        drive = new Drive(driveController, new DriveIO() {});
        break;
    }

    configureBindings();
  }

  private void configureBindings() {}

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        RobotConfig.bumperWidthY.in(Units.Meters),
        RobotConfig.bumperWidthX.in(Units.Meters),
        edu.wpi.first.math.util.Units.inchesToMeters(3.0),
        RobotState.getInstance()::getSimulatedDrivePose,
        RobotState.getInstance()::getFieldVelocity);

    fuelSim.enableAirResistance();
    fuelSim.start();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
