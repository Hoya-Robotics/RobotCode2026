package frc.robot.subsystems;

import frc.robot.subsystems.azimuth.Azimuth;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.StateSubsystem;

enum SuperStructureState {
  IDLE,
  INTAKE,
  SHOOT_AND_INTAKE,
  SHOOT
}

public class SuperStructure extends StateSubsystem<SuperStructureState> {
  private final Spindexer spindexer;
  private final Hood hood;
  private final Azimuth azimuth;
  private final Launcher launcher;
  private final Intake intake;

  public SuperStructure(
      Spindexer spindexer, Hood hood, Azimuth azimuth, Launcher launcher, Intake intake) {
    this.spindexer = spindexer;
    this.hood = hood;
    this.azimuth = azimuth;
    this.launcher = launcher;
    this.intake = intake;
  }

  @Override
  public void periodic() {}

  @Override
  public void applyState() {
    var state = getCurrentState();
    switch (state) {
      case IDLE:
      case INTAKE:
        spindexer.hold();
        // TODO: tracking logic with hood, azimuth, launcher
        if (state == SuperStructureState.INTAKE) intake.run();
        break;
      case SHOOT:
      case SHOOT_AND_INTAKE:
        spindexer.feed();
        // TODO: shooting logic with hood, azimuth, launcher
        if (state == SuperStructureState.SHOOT_AND_INTAKE) {
          intake.run();
        } else {
          intake.retract();
        }
        break;
      default:
        break;
    }
  }
}
