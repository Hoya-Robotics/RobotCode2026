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
  private boolean hasExtended = false;

  private LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/Spin/speedRPM", 1800);
  private double latestIntakeSpeed = 1800;
  private boolean retractHasExtended = false;

  private Timer stateChangeTimer = new Timer();
	private Timer agitateTimer = new Timer();
  private IntakeState weakRequest = null;

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

    if (weakRequest != null && getCurrentState() == IntakeState.IDLE) {
      setState(weakRequest);
    }

    applyState();
    io.applyOutputs(outputs);
  }

  public Command setStateCommand(IntakeState state) {
    return Commands.runOnce(() -> setState(state), this);
  }

  public Command weakSetStateCommand(IntakeState state) {
    return Commands.runOnce(() -> weakRequest = state);
  }

  public Command clearWeakState() {
    return Commands.runOnce(() -> weakRequest = null);
  }

  private boolean isStalled() {
    return stateChangeTimer.get() > 0.5 && inputs.intakeVelocity.abs(RotationsPerSecond) < 2.0;
  }

  @Override
  public IntakeState handleStateTransitions() {
    if (getRequestedState() == IntakeState.RETRACT_SLOW
        && getCurrentState() != IntakeState.RETRACT_SLOW
        && getCurrentState() != IntakeState.IDLE) {
			retractHasExtended = false;
		}
    if (getRequestedState() != getCurrentState() && getCurrentState() != IntakeState.REVERSE) {
      stateChangeTimer.restart();
    }
    return getRequestedState();
  }

  @Override
  public void applyState() {
    if (getCurrentState() == IntakeState.INTAKE && isStalled()) {
      setState(IntakeState.REVERSE);
    }

    outputs.extendControlType = ExtendControlType.POSITION;
    switch (getCurrentState()) {
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
      case RETRACT_SLOW:
        if (!retractHasExtended) {
          retractHasExtended = inputs.extendPosition.gt(Inches.of(10.75));
          outputs.extendControlType = ExtendControlType.POSITION;
          outputs.extendSetpointInches = IntakeConstants.maxExtension.in(Inches);
        } else {
          outputs.extendControlType = ExtendControlType.VOLTAGE;
          outputs.extendVoltage = Timer.getFPGATimestamp() % 1.0 > 0.75 ? 3.5 : -1.0;
        }
        outputs.intakeVelocityRPM = 1800;
        break;
    }

    // First extension override
    if (!hasExtended && DriverStation.isEnabled()) {
      outputs.extendControlType = ExtendControlType.VOLTAGE;
      outputs.extendVoltage = 2.75;

      hasExtended = inputs.extendPosition.gt(Inches.of(10.75));
    }
  }
}
