package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class BatteryLogger {
  public record EnergyState(double current, double power, double energy) {
    public static EnergyState integrate(EnergyState past, EnergyState present) {
      return new EnergyState(present.current(), present.power(), past.energy() + present.energy());
    }

    public EnergyState sum(EnergyState b) {
      return new EnergyState(current + b.current(), power + b.power(), energy + b.energy());
    }
  }

  private EnergyState robotState = new EnergyState(0.0, 0.0, 0.0);
  private Map<String, EnergyState> subsystemStates = new HashMap<>();
  private static final double kLoopPeriod = 1.0 / 50.0;

  public void reportCurrentUsage(String key, double amps) {
    double power = amps * RobotController.getBatteryVoltage();
    EnergyState subsystemState = new EnergyState(amps, power, power * kLoopPeriod);
    robotState = robotState.sum(subsystemState);
    subsystemStates.merge(key, subsystemState, EnergyState::integrate);
  }

  public void periodic() {
    Logger.recordOutput("BatteryLogger/totalCurrent", robotState.current(), "amps");
    Logger.recordOutput("BatteryLogger/totalPower", robotState.power(), "watts");
    Logger.recordOutput("BatteryLogger/totalEnergy", robotState.energy(), "watt seconds");

    for (var entry : subsystemStates.entrySet()) {
      var state = entry.getValue();
      Logger.recordOutput("BatteryLogger/Current/" + entry.getKey(), state.current(), "amps");
      Logger.recordOutput("BatteryLogger/Power/" + entry.getKey(), state.power(), "watts");
      Logger.recordOutput("BatteryLogger/Energy/" + entry.getKey(), state.energy(), "watt hours");
      subsystemStates.put(entry.getKey(), new EnergyState(0.0, 0.0, 0.0));
    }

    robotState = new EnergyState(0.0, 0.0, robotState.energy());
  }
}
