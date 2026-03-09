package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.TurretConstants;
import frc.robot.RobotConfig.TurretTarget;
import frc.robot.RobotState;
import frc.robot.RobotState.TurretState;
import frc.robot.subsystems.azimuth.Azimuth;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum SuperStructureState {
  IDLE,
  INTAKE,
  SHOOT,
  UNJAM
}

public class SuperStructure extends StateSubsystem<SuperStructureState> {
  private final Spindexer spindexer;
  private final Hood hood;
  private final Azimuth azimuth;
  private final Launcher launcher;
  private final Intake intake;
  private TurretTarget target;

  public SuperStructure(
      Spindexer spindexer, Hood hood, Azimuth azimuth, Launcher launcher, Intake intake) {
    this.target = TurretTarget.HUB;
    this.spindexer = spindexer;
    this.hood = hood;
    this.azimuth = azimuth;
    this.launcher = launcher;
    this.intake = intake;

    setState(SuperStructureState.IDLE);
  }

  @Override
  public void periodic() {

    applyState();
  }

  public Command setTarget(TurretTarget target) {
    return Commands.runOnce(() -> this.target = target);
  }

  public Command idle() {
    return Commands.runOnce(() -> setState(SuperStructureState.IDLE));
  }

  public Command intake() {
    return Commands.runOnce(() -> setState(SuperStructureState.INTAKE));
  }

  public Command shoot() {
    return Commands.runOnce(() -> setState(SuperStructureState.SHOOT));
  }

  public Command unjam() {
    return Commands.runOnce(() -> setState(SuperStructureState.UNJAM));
  }

  @Override
  public void applyState() {
    Logger.recordOutput("SuperStructure/state", getCurrentState());
    TurretState turretParams = RobotState.getInstance().resolveTurretTargetting(target);

    azimuth.setAngle(turretParams.azimuthAngle());
    hood.setAngle(turretParams.hoodAngle());

    launcher.setVoltage(TurretConstants.shooterWarmVoltage);
    SuperStructureState state = getCurrentState();
    switch (state) {
      case IDLE:
				intake.retract();
				spindexer.hold();
      case INTAKE:
				intake.run();
				spindexer.hold();
      case UNJAM:
				intake.stay();
				spindexer.reverse();
        break;
      case SHOOT:
        launcher.setVoltage(turretParams.launchVoltage());
        if (azimuth.getAngle().isNear(turretParams.azimuthAngle(), TurretConstants.azimuthTolerance)
            && hood.getAngle().isNear(turretParams.hoodAngle(), TurretConstants.hoodTolerance)
            && launcher.getSpeed().gt(TurretConstants.shotSpeedThreshold)) {
          intake.agitate();
          spindexer.feed();
        }
        break;
    }
  }
}
