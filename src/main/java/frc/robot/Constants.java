package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public record RobotConfig(
      Mass mass,
      Distance bumperWidth,
      Distance bumperLength,
      Distance trackWidth,
      Distance trackLength) {}

  public static final RobotConfig config =
      new RobotConfig(
          Kilograms.of(65.0), Inches.of(37.0), Inches.of(34.0), Inches.of(23), Inches.of(23));

  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }
}
