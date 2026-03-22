// Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or
// share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.DriveConstants;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.RobotConfig.VisionConstants;
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
import frc.robot.util.PhoenixSync;
import java.util.Optional;

public class RobotContainer {
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);
  public final Drive drive;
  public final Spindexer spindexer;
  public final Intake intake;
  public final Azimuth azimuth;
  public final Hood hood;
  public final Launcher launcher;
  public final SuperStructure superStructure;
  public final Vision vision;
  public FuelSim fuelSim = null;

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
        intake = new Intake(new IntakeIOSimSimple());
        azimuth =
            new Azimuth(
                // new AzimuthIOSimAdvanced(TurretConstants.azimuthMotorId,
                // TurretConstants.azimuthEncoderId));
                new AzimuthIOSimSimple());
        hood = new Hood(new HoodIOSimSimple(), azimuth::getAngle);
        launcher = new Launcher(new LauncherIO() {});
        vision = new Vision(new VisionIO() {});
        /*
        vision =
            new Vision(
                new VisionIOSim(
                    new CameraConfig(
                        "limelight-hopper", VisionConstants.hopperRobotToCamera, Optional.empty()),
                    Optional.empty()),
                new VisionIOSim(
                    new CameraConfig(
                        "limelight-turret",
                        VisionConstants.turretRobotToCamera,
                        Optional.of(azimuth::isCameraAccurate)),
                    Optional.of(azimuth::getTurretCameraPose)));*/
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
                new IntakeIOHardware(
                    IntakeConstants.extendMotorId, 32, IntakeConstants.intakeMotorId));
        azimuth =
            new Azimuth(
                new AzimuthIOHardware(
                    TurretConstants.azimuthMotorId, TurretConstants.azimuthEncoderId));
        hood = new Hood(new HoodIOHardware(TurretConstants.hoodMotorId), azimuth::getAngle);
        launcher = new Launcher(new LauncherIOHardware(TurretConstants.launcherMotorId));
        vision =
            new Vision(
                new VisionIOLimelight(
                    new CameraConfig(
                        "limelight-hopper", VisionConstants.hopperRobotToCamera, Optional.empty()),
                    Optional.empty()),
                new VisionIOLimelight(
                    new CameraConfig(
                        "limelight-turret",
                        TurretConstants.robotToTurret.plus(
                            new Transform3d(Translation3d.kZero, TurretConstants.cameraRotation)),
                        Optional.of(azimuth::isCameraAccurate)),
                    Optional.of(azimuth::getTurretCameraPose)));
        break;
      default:
        vision = new Vision(new VisionIO() {});
        drive = new Drive(driveController, new DriveIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        intake = new Intake(new IntakeIO() {});
        azimuth = new Azimuth(new AzimuthIO() {});
        hood = new Hood(new HoodIO() {}, azimuth::getAngle);
        launcher = new Launcher(new LauncherIO() {});
        break;
    }
    superStructure = new SuperStructure(spindexer, hood, azimuth, launcher, intake);
    PhoenixSync.optimizeAll();
    AutoBuilder.registerAutoChoices(drive, superStructure);

    configureBindings();
  }

  private void configureBindings() {
    driveController
        .rightTrigger(0.3)
        .whileTrue(superStructure.intake())
        .onFalse(superStructure.idle());
    /*
    driveController
        .leftTrigger(0.3)*/
    operatorController
        .rightTrigger(0.3)
        .whileTrue(
            superStructure.shoot().andThen(Commands.runOnce(() -> drive.setTeleopSpeedLimit(1.0))))
        .onFalse(
            superStructure
                .idle()
                .andThen(
                    Commands.runOnce(
                        () -> drive.setTeleopSpeedLimit(DriveConstants.maxDriveSpeedMps))));
    /*
    driveController
        .leftTrigger(0.3)*/
    operatorController
        .rightTrigger(0.3)
        .and(driveController.rightTrigger(0.3))
        .whileTrue(
            superStructure
                .setStateCommand(SuperStructureState.SHOOT_INTAKE)
                .andThen(Commands.runOnce(() -> drive.setTeleopSpeedLimit(1.2))))
        .onFalse(
				Commands.either(
					superStructure.shoot().andThen(Commands.runOnce(() -> drive.setTeleopSpeedLimit(1.0))),
					Commands.runOnce(() -> drive.setTeleopSpeedLimit(DriveConstants.maxDriveSpeedMps)).andThen(
						Commands.either(
							superStructure.intake(),
							superStructure .idle(),
							() -> driveController.getRightTriggerAxis() > 0.3
						)
					),
						() -> operatorController.getRightTriggerAxis() > 0.3
				));
    driveController
        .rightBumper()
        .onTrue(superStructure.setStateCommand(SuperStructureState.REVERSE_INTAKE))
        .onFalse(superStructure.setStateCommand(SuperStructureState.IDLE));
    driveController
        .b()
        .onTrue(superStructure.setTarget(TurretTarget.CONSTANT_FORWARD))
        .onFalse(superStructure.setTarget(TurretTarget.DEFAULT));
    driveController
        .a()
        .onTrue(superStructure.setTarget(TurretTarget.MAX_PASSING))
        .onFalse(superStructure.setTarget(TurretTarget.DEFAULT));
    driveController
        .start()
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.resetOdometry(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            Rotation2d.kZero))));
    driveController
        .x()
        .onTrue(superStructure.setTarget(TurretTarget.TUNING))
        .onFalse(superStructure.setTarget(TurretTarget.DEFAULT));
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        RobotConfig.bumperWidthY.times(2.0).in(Units.Meters),
        RobotConfig.bumperWidthX.times(2.0).in(Units.Meters),
        edu.wpi.first.math.util.Units.inchesToMeters(6.5),
        RobotState.getInstance()::getSimulatedPose,
        RobotState.getInstance()::getFieldVelocity);
    fuelSim.registerIntake(
        Inches.of(16.875).in(Meters),
        Inches.of(16.875 + 9.375).in(Meters),
        Inches.of(13.75).unaryMinus().in(Meters),
        Inches.of(13.75).in(Meters),
        () -> superStructure.isIntaking(),
        RobotState.getInstance()::addFuel);
    // fuelSim.enableAirResistance();
    fuelSim.start();

    RobotState.getInstance().registerFuelSim(fuelSim);
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.autoChooser.get();
  }
}
