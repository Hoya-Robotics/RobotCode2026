// Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or
// share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.RobotState.TurretState;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.azimuth.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixSync;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final Drive drive;
  public final Spindexer spindexer;
  public final Intake intake;
  public final Azimuth azimuth;
  public final Hood hood;
  public final Launcher launcher;
  public final SuperStructure superStructure;
  public final Vision vision;
  public FuelSim fuelSim = null;
  private final LoggedDashboardChooser<Command> autoChooser;

  private LoggedTunableNumber hoodAngle = new LoggedTunableNumber("hoodAngleDegrees", 0.0);
  private LoggedTunableNumber launcherVoltage = new LoggedTunableNumber("launcherVoltage", 12.0);

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
        spindexer = new Spindexer(new SpindexerIO() {});
        intake = new Intake(new IntakeIO() {});
        azimuth = new Azimuth(new AzimuthIO() {});
        hood = new Hood(new HoodIO() {});
        launcher = new Launcher(new LauncherIO() {});
        vision = new Vision(new VisionIO() {});
        configureFuelSim();
        break;
      case REAL:
        drive =
            new Drive(
                driveController,
                new DriveIOHardware(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontRight,
                    TunerConstants.FrontLeft,
                    TunerConstants.BackRight,
                    TunerConstants.BackLeft));

        spindexer =
            new Spindexer(
                new SpindexerIOHardware(
                    RobotConfig.SpindexerConstants.indexMotorId,
                    RobotConfig.SpindexerConstants.feedMotorId));
        intake =
            new Intake(
                new IntakeIOHardware(IntakeConstants.extendMotorId, IntakeConstants.intakeMotorId));
        azimuth =
            new Azimuth(
                new AzimuthIOHardware(
                    TurretConstants.azimuthMotorId, TurretConstants.azimuthEncoderId));
        hood = new Hood(new HoodIOHardware(TurretConstants.hoodMotorId));
        launcher = new Launcher(new LauncherIOHardware(TurretConstants.launcherMotorId));
        vision =
            new Vision(
                new VisionIOLimelight(
                    VisionConstants.turretCameraConfig, Optional.of(azimuth::getTurretCameraPose)));
        break;
      default:
        vision = new Vision(new VisionIO() {});
        drive = new Drive(driveController, new DriveIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        intake = new Intake(new IntakeIO() {});
        azimuth = new Azimuth(new AzimuthIO() {});
        hood = new Hood(new HoodIO() {});
        launcher = new Launcher(new LauncherIO() {});
        break;
    }
    Supplier<TurretState> testSetpoints =
        () -> {
          Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
          Translation2d hubPosition = FieldConstants.Hub.getTopCenter().toTranslation2d();
          Rotation2d azimuth = hubPosition.minus(robotPose.getTranslation()).getAngle();
          Rotation2d robotRelativeAngle = azimuth.minus(robotPose.getRotation());

          Logger.recordOutput(
              "Tuning/hubDistance", robotPose.getTranslation().getDistance(hubPosition));

          return new TurretState(
              robotRelativeAngle.getMeasure(),
              Units.Degrees.of(hoodAngle.getAsDouble()),
              launcherVoltage.getAsDouble());
        };
    superStructure = new SuperStructure(testSetpoints, spindexer, hood, azimuth, launcher, intake);
    // superStructure = new SuperStructure(RobotState.getInstance()::getTurretSetpoint, spindexer,
    // hood, azimuth, launcher, intake);
    PhoenixSync.optimizeAll();

    autoChooser = new LoggedDashboardChooser<>("auto choices");
    autoChooser.addDefaultOption(
        "autoBuilderTest",
        AutoBuilder.getTrajectoryCommand(AutoBuilder.testWaypoints, drive, superStructure));

    configureBindings();
  }

  private void configureBindings() {
    driveController.leftBumper().onTrue(superStructure.intake()).onFalse(superStructure.idle());
    driveController.rightTrigger(0.3).onTrue(superStructure.shoot()).onFalse(superStructure.idle());
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        RobotConfig.bumperWidthY.in(Units.Meters),
        RobotConfig.bumperWidthX.in(Units.Meters),
        edu.wpi.first.math.util.Units.inchesToMeters(3.0),
        RobotState.getInstance()::getSimulatedPose,
        RobotState.getInstance()::getFieldVelocity);

    fuelSim.enableAirResistance();
    fuelSim.start();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
