package frc.robot.util;

import frc.robot.util.PhoenixSync.TalonFXSignals;

public record MotorState(
    boolean isConnected,
    double voltageApplied,
    double currentAmps,
    double nativePosition,
    double nativeVelocity,
    double temperature) {
  public static MotorState kZero = new MotorState(false, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static MotorState fromTalonSignals(TalonFXSignals signals) {
    return new MotorState(
        signals.isConnected(),
        signals.getVoltage(),
        signals.getCurrent(),
        signals.getNativePosition(),
        signals.getNativeVelocity(),
        signals.getTemp());
  }
}
