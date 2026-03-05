package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum IntakeState {
  IDLE,
  RETRACT,
  INTAKE
}

public class Intake extends StateSubsystem<IntakeState> {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIOOutputs outputs;

  public Intake(IntakeIO io) {
    this.io = io;
    setState(IntakeState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    applyState();
    io.applyOutputs(outputs);
  }

  public void run() {
    setState(IntakeState.INTAKE);
  }

  public void retract() {
    setState(IntakeState.RETRACT);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case IDLE:
        break;
      case RETRACT:
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
