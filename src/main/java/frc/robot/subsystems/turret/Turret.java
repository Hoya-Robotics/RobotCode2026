package frc.robot.subsystems.turret;

import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.StateSubsystem;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

enum TurretState {
  IDLE,
  TRACK,
  SHOOT
}

public class Turret extends StateSubsystem<TurretState> {
	public static record ShotParameters(Angle	yawAngle, Angle hoodAngle, double flywheelSpeed) {}
	private Supplier<ShotParameters> parameters;

  private final TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private TurretIOOutputs outputs = new TurretIOOutputs();

  public Turret(TurretIO io) {
    this.io = io;

		setState(TurretState.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    applyState();
    io.applyOutputs(outputs);
  }

  @Override
  public void applyState() {
		var params = parameters.get();
    switch (getCurrentState()) {
			case IDLE: break;
			case TRACK:
				outputs.hoodAngle = params.hoodAngle().in(Units.Radian);
				outputs.yawAngle = params.yawAngle().in(Units.Radian);
				outputs.flywheelSpeed = 0.0;
				break;
			case SHOOT:
				outputs.hoodAngle = params.hoodAngle().in(Units.Radian);
				outputs.yawAngle = params.yawAngle().in(Units.Radian);
				outputs.flywheelSpeed = params.flywheelSpeed();
				break;
    }
  }
}
