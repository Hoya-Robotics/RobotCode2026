package frc.robot.subsystems;

import javax.print.attribute.SetOfIntegerSyntax;

import frc.robot.RobotState;
import frc.robot.RobotState.TurretState;
import frc.robot.subsystems.azimuth.Azimuth;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.StateSubsystem;

enum SuperStructureState {
  IDLE,
  INTAKE,
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
		TurretState setpoints = RobotState.getInstance().getTurretSetpoints(
			new TurretState(azimuth.getAngle(), hood.getAngle(), launcher.getSpeed())
		);

		// Always track hub
		azimuth.setAngle(setpoints.azimuthAngle());
		hood.setAngle(setpoints.hoodAngle());

    switch (getCurrentState()) {
			case IDLE:
				intake.retract();
				spindexer.hold();
				break;
			case INTAKE:
				intake.run();
				spindexer.hold();
				break;
			case SHOOT:
				// TODO: Ensure azimuth + hood are within tolerance to shoot
				launcher.setSpeed(setpoints.launchSpeed());
				intake.retract();
				spindexer.feed();
				break;
    }
  }
}
