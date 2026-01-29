package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotConfig;
import frc.robot.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum DriveState {
  IDLE,
  TO_POSE,
  TELEOP
}

public class Drive extends StateSubsystem<DriveState> {
  private final GyroIO gyro;
  private GyroIOInputsAutoLogged gyroData = new GyroIOInputsAutoLogged();

  private final Module[] modules = new Module[4];
  private final XboxController driveController;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(RobotConfig.moduleTranslations);

  public Drive(XboxController controller, GyroIO gyro, ModuleIO[] moduleIOs) {
    this.driveController = controller;
    this.gyro = gyro;
    for (int i = 0; i < 4; ++i) {
      this.modules[i] = new Module(i, moduleIOs[i]);
    }
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroData);
    Logger.processInputs("Gyro", gyroData);

    for (var m : modules) m.periodic();

    statePeriodic();

    for (var m : modules) m.applyOutputs();
  }

  public void runSetpoint(ChassisSpeeds speeds) {
    var moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, RobotConfig.maxDriveSpeedMps);

    for (int i = 0; i < 4; ++i) {
      modules[i].runSetpoint(moduleStates[i]);
    }
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
    }
  }

	/*
  private ChassisSpeeds getControllerSpeeds() {
  }
	*/
}
