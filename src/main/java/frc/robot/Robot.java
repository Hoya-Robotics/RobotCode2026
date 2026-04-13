// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.GenericTunableGains;
import frc.robot.util.PhoenixSync;
import frc.robot.util.ShiftTracker;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  public static final BatteryLogger batteryLogger = new BatteryLogger();
  private final RobotContainer m_robotContainer;
  private Notifier batteryLoggerThread = new Notifier(batteryLogger::periodic);

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
    // batteryLoggerThread.startPeriodic(0.25); // Log currents at lower frequency 4Hz

    m_robotContainer = new RobotContainer();

    Autos.warmup(m_robotContainer.drive, m_robotContainer.superStructure, m_robotContainer.intake);
  }

  @Override
  public void robotInit() {
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  @Override
  public void robotPeriodic() {
    PhoenixSync.refreshCriticalBlocking(0.008);
    PhoenixSync.refreshTelemetryNonBlock();
    GenericTunableGains.periodicAll();
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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    m_robotContainer.vision.captureRewind(20.0);
    m_robotContainer.drive.setIdle();
  }

  @Override
  public void teleopInit() {
    ShiftTracker.start();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    ShiftTracker.run();

    Logger.recordOutput("ShiftTracker/shiftRemainingSeconds", ShiftTracker.timeTillShiftEnds());
    Logger.recordOutput("ShiftTracker/hubActive", ShiftTracker.isHubActive());

    double remaining = ShiftTracker.timeTillShiftEnds();
    if (remaining > 0.0 && remaining <= 10.0) {
      m_robotContainer.driveController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      m_robotContainer.driveController.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

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
    m_robotContainer.drive.resetOdometry(new Pose2d(3.0, 3.0, Rotation2d.kZero));
  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.fuelSim.updateSim();
  }
}
