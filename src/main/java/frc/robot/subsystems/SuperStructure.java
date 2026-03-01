package frc.robot.subsystems;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.StateSubsystem;

enum SuperStructureState {
  IDLE,
  FEED,
}

public class SuperStructure extends StateSubsystem<SuperStructureState> {
  private final Spindexer spindexer;
	private final Turret turret;
	private final Intake intake;

  public SuperStructure(Spindexer spindexer, Turret turret, Intake intake) {
    this.spindexer = spindexer;
		this.turret = turret;
		this.intake = intake;
  }

  @Override
  public void periodic() {
	}

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case IDLE:
        spindexer.hold();
        break;
      case FEED:
        spindexer.feed();
        break;
    }
  }
}
