package frc.robot.subsystems.azimuth;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Azimuth extends SubsystemBase {
  private final AzimuthIO io;
  private final AzimuthIOInputsAutoLogged inputs = new AzimuthIOInputsAutoLogged();

  public Azimuth(AzimuthIO io) {
    this.io = io;
  }

	public Angle getAngle() {
		return Units.Radians.of(inputs.positionRads);
	}

  public void setAngle(Angle angle) {
    this.io.setAngle(angle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Azimuth", inputs);
  }
}
