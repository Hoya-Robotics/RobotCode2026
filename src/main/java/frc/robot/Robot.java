// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.util.FuelSim;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("ProjectName", "Rebuilt4152");
    switch (RobotConfig.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter()); // USB Logging
        Logger.addDataReceiver(new NT4Publisher()); // Network Table Logging
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Network Table Logging
        break;
    }

    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    m_robotContainer.drive.driveToPose(new Pose2d(3.5, 2.5, Rotation2d.k180deg));
    var initialShots =
        new Notifier(
            () -> {
              m_robotContainer.turret.letShoot();
            });
    initialShots.startPeriodic(0.25);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    var fuelSim = FuelSim.getInstance();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        RobotConfig.bumperWidthY.in(Meters),
        RobotConfig.bumperWidthX.in(Meters),
        Units.inchesToMeters(3.0),
        RobotState.getInstance()::getOdometryPose,
        m_robotContainer.drive::getChassisSpeeds);

    fuelSim.start();

    Logger.recordOutput(
        "RobotConfig/cameraTransforms",
        Arrays.stream(RobotConfig.cameras.toArray(CameraConfig[]::new))
            .map(
                c ->
                    new Pose3d(RobotState.getInstance().getOdometryPose())
                        .transformBy(c.robotToCamera()))
            .toArray(Pose3d[]::new));
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    FuelSim.getInstance().updateSim();
  }
}
