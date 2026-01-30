// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.turret.PlaceholderTurret;

public class RobotContainer {
  public final XboxController driveController = new XboxController(0);
  public final Drive drive;
  public final PlaceholderTurret turret = new PlaceholderTurret();

  public RobotContainer() {
    switch (RobotConfig.getMode()) {
      case SIM:
        drive = Drive.simulatedDrive(driveController);
        break;
      default:
        drive = Drive.empty();
        break;
    }

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
