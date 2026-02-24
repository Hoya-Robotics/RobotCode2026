package frc.robot.subsystems;

import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.StateSubsystem;

enum SuperStructureState {
	IDLE,
	FEED,
}

public class SuperStructure extends StateSubsystem<SuperStructureState> {
	private final Spindexer spindexer;

	public SuperStructure(Spindexer spindexer) {
		this.spindexer = spindexer;
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
