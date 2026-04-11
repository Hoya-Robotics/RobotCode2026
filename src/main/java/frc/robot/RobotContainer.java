// Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or
// share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.RobotConfig.IntakeConstants.IntakeState;
import frc.robot.RobotConfig.SuperStructureState;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.VisionConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOHardware;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOLimelightSim;
import frc.robot.util.FuelSim;
import frc.robot.util.PhoenixSync;

public class RobotContainer {
  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);
  public final Drive drive;
  public final Spindexer spindexer;
  public final Intake intake;
  public final Turret turret;
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
        turret = new Turret(new TurretIOSim());
        vision =
            new Vision(
                new VisionIOLimelightSim(VisionConstants.turretConfig),
                new VisionIOLimelightSim(VisionConstants.hopperConfigSim));
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
                    RobotConfig.SpindexerConstants.feedMotorId,
                    RobotConfig.SpindexerConstants.rampMotorId));
        intake =
            new Intake(
                new IntakeIOHardware(
                    IntakeConstants.extendMotorId,
                    IntakeConstants.intakeEncoderId,
                    IntakeConstants.intakeMotorId));
        turret =
            new Turret(
                new TurretIOHardware(
                    TurretConstants.azimuthMotorId,
                    TurretConstants.azimuthEncoderId,
                    TurretConstants.hoodMotorId,
                    TurretConstants.launcherMotorId));
        vision =
            new Vision(
                new VisionIOLimelight(VisionConstants.turretConfig),
                new VisionIOLimelight(VisionConstants.hopperConfig));
        break;
      default:
        vision = new Vision(new VisionIO() {});
        drive = new Drive(driveController, new DriveIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        turret = new Turret(new TurretIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }
    superStructure = new SuperStructure(spindexer, turret, intake);
    PhoenixSync.optimizeAll();

    configureBindings();
  }

  private void configureBindings() {
    superStructure.setDefaultCommand(superStructure.setStateCommand(SuperStructureState.IDLE));
    driveController
        .rightTrigger(0.3)
        .and(driveController.leftTrigger(0.3).negate())
        .whileTrue(superStructure.setStateCommand(SuperStructureState.INTAKE).repeatedly());
    // operatorController
    driveController
        // .rightTrigger(0.3)
        .leftTrigger(0.3)
        .and(driveController.rightTrigger(0.3).negate())
        .whileTrue(superStructure.setStateCommand(SuperStructureState.SHOOT).repeatedly());
    // operatorController
    driveController
        // .rightTrigger(0.3)
        .leftTrigger(0.3)
        .and(driveController.rightTrigger(0.3))
        .whileTrue(superStructure.setStateCommand(SuperStructureState.SHOOT_INTAKE).repeatedly());

    driveController
        .rightBumper()
        .whileTrue(superStructure.setStateCommand(SuperStructureState.REVERSE_INTAKE).repeatedly());

    /*
     driveController
         .b()
         .whileTrue(superStructure.setTargetCommand(TurretTarget.CONSTANT_FORWARD))
         .onFalse(superStructure.setTargetCommand(TurretTarget.DEFAULT));
     driveController
         .x()
         .whileTrue(superStructure.setTargetCommand(TurretTarget.TUNING))
         .onFalse(superStructure.setTargetCommand(TurretTarget.DEFAULT));
    */

    driveController
        .start()
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.resetOdometry(
                        new Pose2d(
                            RobotState.getInstance().getEstimatedPose().getTranslation(),
                            Rotation2d.kZero))));
  }

  private void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        RobotConfig.bumperWidthY.times(2.0).in(Units.Meters),
        RobotConfig.bumperWidthX.times(2.0).in(Units.Meters),
        edu.wpi.first.math.util.Units.inchesToMeters(6.5),
        () ->
            RobotState.getInstance()
                .getSimulatedPose()
                .transformBy(
                    new Transform2d(
                        TurretConstants.robotToTurret.getTranslation().toTranslation2d(),
                        Rotation2d.kZero)),
        RobotState.getInstance()::getFieldVelocity);
    fuelSim.registerIntake(
        Inches.of(16.875).in(Meters),
        Inches.of(16.875 + 9.375).in(Meters),
        Inches.of(13.75).unaryMinus().in(Meters),
        Inches.of(13.75).in(Meters),
        () -> intake.getCurrentState() == IntakeState.INTAKE,
        RobotState.getInstance()::addFuel);
    fuelSim.enableAirResistance();
    fuelSim.start();

    RobotState.getInstance().registerFuelSim(fuelSim);
  }

  public Command getAutonomousCommand() {
    return Autos.getAuto();
  }
}
