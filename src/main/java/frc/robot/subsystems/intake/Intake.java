package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotConfig.IntakeConstants;
import frc.robot.RobotConfig.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.StateSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateSubsystem<IntakeState> {
  private final IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeIOOutputs outputs = new IntakeIOOutputs();
  // private boolean agitatingForward = false;
  private boolean hasExtended = false;

  private LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/Spin/speedRPM", 3200);
  // private LoggedTunableNumber agitateOut = new LoggedTunableNumber("Intake/Rack/agitoutOutIn",
  // 9.0);
  // private LoggedTunableNumber agitateIn = new LoggedTunableNumber("Intake/Rack/agitoutInIn",
  // 5.1);

  private double latestIntakeSpeed = 3200;

  private Timer stateChangeTimer = new Timer();
  private Timer agitateTimer = new Timer();

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

    applyState();
    io.applyOutputs(outputs);
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
        outputs.intakeVelocity = RPM.of(750);
        break;
      case REVERSE:
        outputs.extensionDistance = IntakeConstants.maxExtension;
        outputs.intakeVelocity = RPM.of(-latestIntakeSpeed);
        break;
      case INTAKE:
        outputs.extensionDistance = IntakeConstants.maxExtension;
        outputs.intakeVelocity = RPM.of(latestIntakeSpeed);
        break;
      case AGITATE:
        outputs.extensionDistance = Inches.of(3.5);
        outputs.intakeVelocity = RPM.of(0.0);
        /*
        if (agitateTimer.get() >= 0.4) {
          agitatingForward = !agitatingForward;
          agitateTimer.restart();
        }

        outputs.extensionDistance =
            Inches.of(agitatingForward ? agitateOut.getAsDouble() : agitateIn.getAsDouble());
        outputs.intakeVelocity = RPM.of(750);*/
        break;
    }

    // First extension override
    if (!hasExtended && DriverStation.isEnabled()) {
      outputs.extensionDistance = IntakeConstants.maxExtension;
      hasExtended = inputs.extendPosition.gt(Inches.of(10.75));
    }
  }
}
