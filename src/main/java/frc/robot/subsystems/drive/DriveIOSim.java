package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConfig;
import frc.robot.util.MapleSimSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

public class DriveIOSim extends DriveIOHardware {
  public MapleSimSwerveDrivetrain mapleSimSwerve = null;
  private Notifier simThread = null;

  public DriveIOSim(
      SwerveDrivetrainConstants constants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    super(constants, regulateForSim(moduleConstants));

    // This might be the problem
    /*
    RobotState.getInstance()
        .addSimPoseSupplier(mapleSimSwerve.mapleSimDrive::getSimulatedDriveTrainPose);*/
    startSimThread(moduleConstants);
  }

  private static SwerveModuleConstants[] regulateForSim(SwerveModuleConstants... moduleConstants) {
    MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(moduleConstants);
    return moduleConstants;
  }

  public void startSimThread(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    mapleSimSwerve =
        new MapleSimSwerveDrivetrain(
            Units.Seconds.of(RobotConfig.SimConstants.drivetrainSimLoopPeriod),
            RobotConfig.robotMass,
            RobotConfig.bumperWidthX,
            RobotConfig.bumperWidthY,
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            1.2,
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            moduleConstants);
    simThread = new Notifier(mapleSimSwerve::update);
    simThread.startPeriodic(RobotConfig.SimConstants.drivetrainSimLoopPeriod);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    Logger.recordOutput(
        "Drive/simulatedPose", mapleSimSwerve.mapleSimDrive.getSimulatedDriveTrainPose());
    super.updateInputs(inputs);
  }

  @Override
  public void resetOdometry(Pose2d override) {
    mapleSimSwerve.mapleSimDrive.setSimulationWorldPose(override);
    Timer.delay(0.05);
    super.resetOdometry(override);
  }
}
