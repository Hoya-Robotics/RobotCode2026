package frc.robot.util;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class GenericTunableGains {
  private final String tableKey;
  protected final Map<String, LoggedTunableNumber> gainEntries = new HashMap<>();
  private Optional<Consumer<Map<String, Double>>> gainConsumer = Optional.empty();
  private static List<GenericTunableGains> instances;

  public GenericTunableGains(String tableKey) {
    if (instances == null) {
      instances = new ArrayList<>();
    }
    this.tableKey = tableKey;
    instances.add(this);
  }

  public static void periodicAll() {
    if (instances != null) {
      instances.forEach(GenericTunableGains::periodic);
    }
  }

  protected void registerUpdateCallback(Consumer<Map<String, Double>> fn) {
    gainConsumer = Optional.of(fn);
  }

  public void registerGain(String key, double value) {
    gainEntries.put(key, new LoggedTunableNumber(tableKey + key, value));
  }

  public void periodic() {
    boolean triggerUpdate =
        gainEntries.entrySet().stream().anyMatch(e -> e.getValue().hasChanged(tableKey.hashCode()));
    if (triggerUpdate) {
      gainConsumer.ifPresent(
          update ->
              update.accept(
                  gainEntries.entrySet().stream()
                      .collect(
                          Collectors.toMap(e -> e.getKey(), e -> e.getValue().getAsDouble()))));
    }
  }

  public static class TalonTunableGains extends GenericTunableGains {
    private final int slotNumber;
    private Optional<TalonFX> motor = Optional.empty();
    private static final Map<String, BiConsumer<SlotConfigs, Double>> gainSetters =
        Map.of(
            "kp", (c, v) -> c.kP = v,
            "ki", (c, v) -> c.kI = v,
            "kd", (c, v) -> c.kD = v,
            "kv", (c, v) -> c.kV = v,
            "ka", (c, v) -> c.kA = v,
            "ks", (c, v) -> c.kS = v);

    public TalonTunableGains(String tableKey, int slotNumber) {
      super(tableKey);
      this.slotNumber = slotNumber;
      super.registerUpdateCallback(this::applyGains);
    }

    public void registerMotor(TalonFX motor) {
      this.motor = Optional.of(motor);
    }

    private void applyGains(Map<String, Double> newGains) {
      motor.ifPresent(
          talon -> {
            var slot = new SlotConfigs();
            slot.SlotNumber = slotNumber;
            talon.getConfigurator().refresh(slot);
            newGains.forEach(
                (k, v) -> {
                  var setter = gainSetters.get(k);
                  if (setter != null) setter.accept(slot, v);
                });
            talon.getConfigurator().apply(slot);
          });
    }
  }

  public static class SparkTunableGains extends GenericTunableGains {
    private Optional<SparkFlex> motor = Optional.empty();
    private static final Map<String, BiConsumer<ClosedLoopConfig, Double>> gainSetters =
        Map.of(
            "kp",
            (c, v) -> c.p(v),
            "ki",
            (c, v) -> c.i(v),
            "kd",
            (c, v) -> c.d(v),
            "kv",
            (c, v) -> c.feedForward.kV(v),
            "ka",
            (c, v) -> c.feedForward.kA(v),
            "ks",
            (c, v) -> c.feedForward.kS(v),
            "cruiseVelocity",
            (c, v) -> c.maxMotion.cruiseVelocity(v),
            "maxAcceleration",
            (c, v) -> c.maxMotion.maxAcceleration(v));

    public SparkTunableGains(String tableKey) {
      super(tableKey);
      super.registerUpdateCallback(this::applyGains);
    }

    public void registerMotor(SparkFlex motor) {
      this.motor = Optional.of(motor);
    }

    private void applyGains(Map<String, Double> newGains) {
      motor.ifPresent(
          spark -> {
            var closedLoop = new ClosedLoopConfig();
            newGains.forEach(
                (k, v) -> {
                  var setter = gainSetters.get(k);
                  if (setter != null) setter.accept(closedLoop, v);
                });
            var config = new SparkFlexConfig();
            config.apply(closedLoop);
            spark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
          });
    }
  }
}
