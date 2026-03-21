package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

public class BatteryLogger {
  public record EnergyState(double current, double power, double energy) {}

  private double batteryVoltage = 12.6;
  private static final double kLoopPeriodSecs = 1 / 50.0;
  private Map<String, EnergyState> subsystemStates = new HashMap<>();

  public void reportCurrentUsage(String key, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += amp;
    double power = totalAmps * batteryVoltage;
    double energy = power * kLoopPeriodSecs;

    subsystemStates.put(key, new EnergyState(totalAmps, power, energy));
  }
}
