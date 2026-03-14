package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

enum IntakeState {
  IDLE,
  RETRACT,
  INTAKE,
  AGITATE,
  REVERSE
}

public class Intake extends StateSubsystem<IntakeState> {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIOOutputs outputs = new IntakeIOOutputs();
  private boolean agitatingForward = false;

  private Timer stateChangeTimer = new Timer();

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

  public void reverse() {
    setState(IntakeState.REVERSE);
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

  public void agitate() {
    setState(IntakeState.AGITATE);
  }

  private boolean isStalled() {
    return stateChangeTimer.get() < 0.5 && inputs.intakeVelocity.abs(RotationsPerSecond) < 2.0;
  }

  @Override
  public IntakeState handleStateTransitions() {
    if (getRequestedState() != getCurrentState()) {
      stateChangeTimer.restart();
    }
    return getRequestedState();
  }

  @Override
  public void applyState() {
    if (getCurrentState() == IntakeState.INTAKE && isStalled()) {
      setState(IntakeState.REVERSE);
    }
    switch (getCurrentState()) {
      case IDLE:
        outputs.extendVoltage = Volts.zero();
        outputs.intakeVoltage = Volts.zero();
        break;
      case RETRACT:
        outputs.extendVoltage = Volts.of(-4.0);
        outputs.intakeVoltage = Volts.of(1.5);
        break;
      case REVERSE:
        outputs.extendVoltage = Volts.of(8.0);
        outputs.intakeVoltage = Volts.of(-9.0);
        break;
      case INTAKE:
        outputs.extendVoltage = Volts.of(8.0);
        outputs.intakeVoltage = Volts.of(9.0);
        break;
      case AGITATE:
        outputs.extendVoltage = Volts.of(6.0 * (agitatingForward ? 1.0 : -1.0));
        agitatingForward =
            agitatingForward
                ? inputs.extendPosition.lt(IntakeConstants.agitateOutDist)
                : inputs.extendPosition.lt(IntakeConstants.agitateInDist);
        outputs.intakeVoltage = Volts.of(2.0);
        break;
    }

    if (inputs.extendPosition.lt(IntakeConstants.maxRetraction.plus(Inches.of(0.25)))) {
      outputs.intakeVoltage = Volts.of(0.0);
    }
  }
}
