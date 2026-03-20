package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
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
  private Timer agitateTimer = new Timer();

  public Intake(IntakeIO io) {
    this.io = io;
    setState(IntakeState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/state", getCurrentState());
    Distance intakePostion = inputs.extendPosition;
    Distance intakeX = Meters.of(Math.sin(8.0) * intakePostion.in(Meters));
    Distance intakeZ = Meters.of(Math.cos(8.0) * intakePostion.in(Meters));
    Logger.recordOutput(
        "Intake/IntakePose", new Pose3d(intakeX, Meters.zero(), intakeZ, Rotation3d.kZero));

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
    return stateChangeTimer.get() > 0.5 && inputs.intakeVelocity.abs(RotationsPerSecond) < 2.0;
  }

  @Override
  public IntakeState handleStateTransitions() {
    if (getRequestedState() != getCurrentState() && getCurrentState() != IntakeState.REVERSE) {
      stateChangeTimer.restart();
    }
    if (getRequestedState() == IntakeState.AGITATE) {
      agitateTimer.start();
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
        outputs.extensionDistance = Inches.of(7.25);
        outputs.intakeVoltage = Volts.zero();
        break;
      case RETRACT:
        outputs.extensionDistance = Inches.of(7.25);
        outputs.intakeVoltage = Volts.of(1.5);
        break;
      case REVERSE:
        outputs.extensionDistance = IntakeConstants.maxExtension;
        outputs.intakeVoltage = Volts.of(-9.0);
        break;
      case INTAKE:
        outputs.extensionDistance = IntakeConstants.maxExtension;
        outputs.intakeVoltage = Volts.of(9.0);
        break;
      case AGITATE:
        if (agitateTimer.get() >= 0.3) {
          agitatingForward = !agitatingForward;
          agitateTimer.restart();
        }
        outputs.extensionDistance =
            agitatingForward ? Inches.of(10.75) : Inches.of(7.25); // 10.75 and 7.75
        outputs.intakeVoltage = Volts.of(2.0);
        break;
    }

    if (inputs.extendPosition.lt(IntakeConstants.maxRetraction.plus(Inches.of(0.25)))) {
      outputs.intakeVoltage = Volts.of(1.0);
    }
  }
}
