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

  private static final List<BaseStatusSignal> allSignals = new ArrayList<>();
  private static final List<ParentDevice> devices = new ArrayList<>();
  private static BaseStatusSignal[] allSignalsArray = null;

  public static TalonFXSignals registerTalonFX(TalonFX motor, double updateFreqHz) {
    var signals =
        new TalonFXSignals(
            motor.getPosition(),
            motor.getVelocity(),
            motor.getAcceleration(),
            motor.getMotorVoltage(),
            motor.getDeviceTemp(),
            motor.getSupplyCurrent());
    new StatusSignalCollection(
            signals.position(),
            signals.velocity(),
            signals.acceleration(),
            signals.voltage(),
            signals.temp(),
            signals.current())
        .setUpdateFrequencyForAll(updateFreqHz);
    allSignals.add(signals.position());
    allSignals.add(signals.velocity());
    allSignals.add(signals.acceleration());
    allSignals.add(signals.voltage());
    allSignals.add(signals.temp());
    allSignals.add(signals.current());
    devices.add(motor);
    allSignalsArray = null; // invalidate cache

    return signals;
  }

  public static void optimizeAll() {
    ParentDevice.optimizeBusUtilizationForAll(devices.toArray(ParentDevice[]::new));
  }

  public static void refreshAll(double timeoutSec) {
    if (allSignalsArray == null) {
      allSignalsArray = allSignals.toArray(new BaseStatusSignal[0]);
    }
    BaseStatusSignal.waitForAll(timeoutSec, allSignalsArray);
  }
}
