package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

enum IntakeState {
	IDLE,
	INSIDE,
	INTAKE
}

public class Intake extends StateSubsystem<IntakeState> {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
	private IntakeIOOutputs outputs;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    applyState();
		io.applyOutputs(outputs);
  }

  @Override
  public void applyState() {
		switch (getCurrentState()) {
			case IDLE:
				outputs.extendMeters = 0.0;
				outputs.spinVoltage = 4.0;
				break;
			case INSIDE:
				outputs.extendMeters = 0.0;
				outputs.spinVoltage = 0.0;
				break;
			case INTAKE:
				outputs.extendMeters = Units.inchesToMeters(5);
				outputs.spinVoltage = 4.0;
				break;
		}
	}
}
