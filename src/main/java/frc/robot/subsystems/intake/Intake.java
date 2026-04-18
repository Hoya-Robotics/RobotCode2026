package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.RobotConfig.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeIO.ExtendControlType;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateSubsystem<IntakeState> {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIOOutputs outputs = new IntakeIOOutputs();

  private boolean deployKicker = true;
  private boolean resettingKicker = false;
  private boolean postDelay = false;
  private Timer deployDelay = new Timer();

  private LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/Spin/speedRPM", 1800);
  private double latestIntakeSpeed = 1800;

  private Timer stateChangeTimer = new Timer();

  private boolean agitatingForward = true;

  public Intake(IntakeIO io) {
    this.io = io;
    setState(IntakeState.IDLE);
  }

  private void logMechanism() {
    Distance intakePostion = inputs.extendPosition;
    Distance intakeX = Meters.of(Math.sin(8.0) * intakePostion.in(Meters));
    Distance intakeZ = Meters.of(Math.cos(8.0) * intakePostion.in(Meters));
    Logger.recordOutput(
        "Intake/IntakePose", new Pose3d(intakeX, Meters.zero(), intakeZ, Rotation3d.kZero));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/intakeVelocityRPM", inputs.intakeVelocity.in(RPM));
    Logger.recordOutput("Intake/state", getCurrentState());

    logMechanism();

    Robot.batteryLogger.reportCurrentUsage("Intake/spin", inputs.intakeCurrent.in(Amps));
    Robot.batteryLogger.reportCurrentUsage("Intake/extend", inputs.extendCurrent.in(Amps));

    if (intakeSpeed.hasChanged(intakeSpeed.hashCode())) {
      latestIntakeSpeed = intakeSpeed.getAsDouble();
    }

    Logger.recordOutput("Intake/deployingKicker", deployKicker);
    if (deployKicker && DriverStation.isEnabled()) {
      setState(resettingKicker ? IntakeState.KICKER_RESET : IntakeState.KICKER_DEPLOY);
    }

    applyState();
    io.applyOutputs(outputs);
  }

  public Command setStateCommand(IntakeState state) {
    return Commands.runOnce(() -> setState(state), this);
  }

  @Override
  public IntakeState handleStateTransitions() {
    if (getRequestedState() != getCurrentState() && getCurrentState() != IntakeState.REVERSE) {
      stateChangeTimer.restart();
    }
    return getRequestedState();
  }

  @Override
  public void applyState() {
    outputs.extendControlType = ExtendControlType.POSITION;

    switch (getCurrentState()) {
      case KICKER_DEPLOY:
        deployKicker = !(postDelay && deployDelay.get() > 0.175);
        resettingKicker = inputs.extendCurrent.gt(Amps.of(15.0));

        if (!postDelay && inputs.extendPosition.gt(Inches.of(4.0))) {
          postDelay = true;
          deployDelay.restart();
        }

        outputs.extendSetpointInches = 4.45;
        break;
      case KICKER_RESET:
        resettingKicker = !inputs.extendPosition.lt(Inches.of(5.75));

        outputs.extendControlType = ExtendControlType.VOLTAGE;
        outputs.extendVoltage = -2.25;
        break;
      case IDLE:
        outputs.extendSetpointInches = 7.0;
        outputs.intakeVelocityRPM = 750;
        break;
      case REVERSE:
        outputs.extendSetpointInches = IntakeConstants.maxExtension.in(Inches);
        outputs.intakeVelocityRPM = -latestIntakeSpeed;
        break;
      case INTAKE:
        outputs.extendSetpointInches = IntakeConstants.maxExtension.in(Inches);
        outputs.intakeVelocityRPM = latestIntakeSpeed;
        break;
      case AGITATE:
        if (agitatingForward) {
          agitatingForward = !inputs.extendPosition.gt(Inches.of(10.5));
          outputs.extendSetpointInches = IntakeConstants.maxExtension.in(Inches);
        } else {
          agitatingForward = inputs.extendPosition.lt(Inches.of(4.5));
          outputs.extendControlType = ExtendControlType.VOLTAGE;
          outputs.extendVoltage = -3.0;
        }
        outputs.intakeVelocityRPM = 1600;
        break;
      case RETRACT:
        // outputs.extendControlType = ExtendControlType.VOLTAGE;
        // outputs.extendVoltage = -2.0;
        outputs.extendSetpointInches = 2.0;
        outputs.intakeVelocityRPM = 1000;
        break;
    }
  }
}
