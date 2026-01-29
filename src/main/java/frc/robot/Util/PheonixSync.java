package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.traits.CommonDevice;

public class PheonixSync {
  private static StatusSignalCollection canivoreSignals = new StatusSignalCollection();

  // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html#canivore-timesync
  public static void registerOptimizedDeviceSignals(
      CommonDevice device, double updateFreqHz, BaseStatusSignal... signals) {
    BaseStatusSignal.setUpdateFrequencyForAll(updateFreqHz, signals);
    canivoreSignals.addSignals(signals);
    device.optimizeBusUtilization();
  }

  public static void refresh() {
    canivoreSignals.refreshAll();
  }
}
