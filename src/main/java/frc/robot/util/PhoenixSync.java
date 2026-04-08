package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
      StatusSignal<Current> current) {

    public double getNativePosition() {
      return BaseStatusSignal.getLatencyCompensatedValueAsDouble(position, velocity);
    }

    public double getNativeVelocity() {
      return BaseStatusSignal.getLatencyCompensatedValueAsDouble(velocity, acceleration);
    }

    public double getNativeAcceleration() {
      return acceleration().getValueAsDouble();
    }

    public double getVoltage() {
      return voltage.getValueAsDouble();
    }

    public double getTemp() {
      return temp.getValueAsDouble();
    }

    public double getCurrent() {
      return current.getValueAsDouble();
    }

    public boolean isConnected() {
      return position.getStatus().isOK();
    }
  }

  private static final List<BaseStatusSignal> telemetrySignals = new ArrayList<>();
  private static final List<BaseStatusSignal> criticalSignals = new ArrayList<>();
  private static final List<ParentDevice> devices = new ArrayList<>();
  private static BaseStatusSignal[] criticalArray = null;
  private static BaseStatusSignal[] telemetryArray = null;

  public static TalonFXSignals registerTalonFX(TalonFX motor, double updateFreqHz) {
    var signals =
        new TalonFXSignals(
            motor.getPosition(),
            motor.getVelocity(),
            motor.getAcceleration(),
            motor.getMotorVoltage(),
            motor.getDeviceTemp(),
            motor.getTorqueCurrent());
    signals.position().setUpdateFrequency(updateFreqHz);
    signals.velocity().setUpdateFrequency(updateFreqHz);
    signals.acceleration().setUpdateFrequency(updateFreqHz);

    signals.voltage().setUpdateFrequency(10);
    signals.current().setUpdateFrequency(10);
    signals.temp().setUpdateFrequency(4);

    criticalSignals.add(signals.position());
    criticalSignals.add(signals.velocity());
    criticalSignals.add(signals.acceleration());

    telemetrySignals.add(signals.voltage());
    telemetrySignals.add(signals.temp());
    telemetrySignals.add(signals.current());

    devices.add(motor);
    criticalArray = null;
    telemetryArray = null;

    return signals;
  }

  public static void optimizeAll() {
    ParentDevice.optimizeBusUtilizationForAll(devices.toArray(ParentDevice[]::new));
  }

  public static void refreshCriticalBlocking(double timeoutSec) {
    if (criticalArray == null) {
      criticalArray = criticalSignals.toArray(new BaseStatusSignal[0]);
    }
    BaseStatusSignal.waitForAll(timeoutSec, criticalArray);
  }

  public static void refreshTelemetryNonBlock() {
    if (telemetryArray == null) {
      telemetryArray = telemetrySignals.toArray(new BaseStatusSignal[0]);
    }
    BaseStatusSignal.refreshAll(telemetryArray);
  }
}
