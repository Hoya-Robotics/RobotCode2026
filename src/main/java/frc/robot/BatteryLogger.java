package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class BatteryLogger {
  private double robotCurrent, robotPower, robotEnergy = 0.0;
  private Map<String, Double> subsystemCurrents = new HashMap<>();
  private Map<String, Double> subsystemPowers = new HashMap<>();
  private Map<String, Double> subsystemEnergies = new HashMap<>();
  private static final double kLoopPeriod = 1.0 / 50.0;

  public void reportCurrentUsage(String key, double amps) {
    double power = amps * RobotController.getBatteryVoltage();
    double energy = power * kLoopPeriod;

    robotCurrent += amps;
    robotPower += power;
    robotEnergy += energy;

    subsystemCurrents.put(key, amps);
    subsystemPowers.put(key, power);
    subsystemEnergies.merge(key, energy, Double::sum);
  }

  public void periodic() {
    Logger.recordOutput("BatteryLogger/totalCurrent", robotCurrent, "amps");
    Logger.recordOutput("BatteryLogger/totalPower", robotPower, "watts");
    Logger.recordOutput("BatteryLogger/totalEnergy", robotEnergy, "watt seconds");

    for (var entry : subsystemCurrents.entrySet()) {
      String key = entry.getKey();
      Logger.recordOutput("BatteryLogger/Current/" + key, subsystemCurrents.get(key), "amps");
      Logger.recordOutput("BatteryLogger/Power/" + key, subsystemPowers.get(key), "watts");
      Logger.recordOutput("BatteryLogger/Energy/" + key, subsystemEnergies.get(key), "watt hours");

      subsystemCurrents.put(key, 0.0);
      subsystemPowers.put(key, 0.0);
    }

    robotCurrent = 0.0;
    robotPower = 0.0;
  }
}
