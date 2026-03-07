package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

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
  private IntakeIOOutputs outputs = new IntakeIOOutputs();

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

  public void stay() {
    setState(IntakeState.IDLE);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case IDLE:
        outputs.extendVoltage = Volts.zero();
        outputs.intakeVoltage = Volts.zero();
        break;
      case RETRACT:
        // TODO: run intake slowly while retracting, unstuck balls
        outputs.extendVoltage = Volts.of(-8.0);
        outputs.intakeVoltage = Volts.of(1.5);
        break;
      case INTAKE:
        outputs.extendVoltage = Volts.of(8.0);
        if (inputs.extendPosition.gt(Inches.of(5.5))) {
          outputs.intakeVoltage = Volts.of(10.0);
        } else {
          outputs.intakeVoltage = Volts.zero();
        }
        break;
    }
  }
}
