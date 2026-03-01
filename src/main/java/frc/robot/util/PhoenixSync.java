package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import java.util.ArrayList;
import java.util.List;

public class PhoenixSync {
  public static record TalonFXSignals(
      StatusSignal<Angle> position,
      StatusSignal<AngularVelocity> velocity,
			StatusSignal<AngularAcceleration> acceleration,
      StatusSignal<Voltage> voltage,
			StatusSignal<Temperature> temp,
			StatusSignal<Current> current
	) {
		public double getPositionRads() {
			return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
		}
		public double getVelocityRadsPerSec() {
			return BaseStatusSignal.getLatencyCompensatedValueAsDouble(velocity, acceleration);
		}
		public double getVoltage() { return voltage.getValueAsDouble(); }
		public double getTempCelsius() { return temp.getValueAsDouble(); }
		public double getCurrentAmps() { return current.getValueAsDouble(); }
		public boolean isConnected() { return position.getStatus().isOK(); }
	}

  private static final List<StatusSignalCollection> signalGroups = new ArrayList<>();
  private static final List<ParentDevice> devices = new ArrayList<>();

  public static TalonFXSignals registerTalonFX(TalonFX motor, double updateFreqHz) {
		var signals = new TalonFXSignals(
			motor.getPosition(),
			motor.getVelocity(),
			motor.getAcceleration(),
			motor.getMotorVoltage(),
			motor.getDeviceTemp(),
			motor.getSupplyCurrent()
		);
    StatusSignalCollection group = new StatusSignalCollection(
			signals.position(), signals.velocity(), signals.acceleration(),
			signals.voltage(), signals.temp(), signals.current()
		);
    group.setUpdateFrequencyForAll(updateFreqHz);
    signalGroups.add(group);
    devices.add(motor);

		return signals;
  }

  public static void optimizeAll() {
    ParentDevice.optimizeBusUtilizationForAll(devices.toArray(ParentDevice[]::new));
  }

  public static void refreshAll(double timeoutSec) {
    for (var group : signalGroups) {
      group.waitForAll(timeoutSec);
    }
  }
}
