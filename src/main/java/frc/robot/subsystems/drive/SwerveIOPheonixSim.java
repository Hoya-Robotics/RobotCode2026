package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

class TalonFXSimWrapper implements SimulatedMotorController {
  private final TalonFXSimState state;

  public TalonFXSimWrapper(TalonFX talon) {
    state = talon.getSimState();
  }

  @Override
  public Voltage updateControlSignal(
      Angle mechanismAngle,
      AngularVelocity mechanismVelocity,
      Angle encoderAngle,
      AngularVelocity encoderVelocity) {
    state.setRawRotorPosition(encoderAngle);
    state.setRotorVelocity(encoderVelocity);
    state.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    return state.getMotorVoltageMeasure();
  }
}

class TalonFXSimWrapperWithEncoder extends TalonFXSimWrapper {
  private final CANcoderSimState encoderState;

  public TalonFXSimWrapperWithEncoder(TalonFX talon, CANcoder encoder) {
    super(talon);
    this.encoderState = encoder.getSimState();
  }

  @Override
  public Voltage updateControlSignal(
      Angle mechanismAngle,
      AngularVelocity mechanismVelocity,
      Angle encoderAngle,
      AngularVelocity encoderVelocity) {
    encoderState.setRawPosition(mechanismAngle);
    encoderState.setVelocity(mechanismVelocity);

    return super.updateControlSignal(
        mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
  }
}

public class SwerveIOPheonixSim extends SwerveIOPheonix {
  private final Pigeon2SimState pigeonSim;
  private final SwerveDriveSimulation mapleDriveSim;
  private final SwerveModuleSimulation[] simModules = new SwerveModuleSimulation[4];

  public SwerveIOPheonixSim(
      SwerveDrivetrainConstants constants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    super(constants, moduleConstants);

    this.pigeonSim = this.getPigeon2().getSimState();
    this.mapleDriveSim =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(1.0, 1.0, Rotation2d.kZero));
    this.resetPose(new Pose2d(1.0, 1.0, Rotation2d.kZero));
    for (int i = 0; i < 4; ++i) {
      var module = this.getModule(i);
      var simModule = mapleDriveSim.getModules()[i];
      simModule.useDriveMotorController(new TalonFXSimWrapper((TalonFX) module.getDriveMotor()));
      simModule.useSteerMotorController(
          new TalonFXSimWrapperWithEncoder(
              (TalonFX) module.getSteerMotor(), (CANcoder) module.getEncoder()));
      this.simModules[i] = simModule;
    }

    SimulatedArena.getInstance().addDriveTrainSimulation(mapleDriveSim);
  }

  public static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      [] regulateModuleConstantsForSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
          moduleConstants) {
    for (var mc : moduleConstants) regulateModuleConfigForSim(mc);
    return moduleConstants;
  }

  private static void regulateModuleConfigForSim(SwerveModuleConstants<?, ?, ?> moduleConstants) {
    moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(70)
                .withKI(0)
                .withKD(4.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withSteerMotorGearRatio(16.0)
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.05))
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }

  @Override
  public void updateModuleInputs(ModuleIOInputs[] inputs) {
    for (int i = 0; i < 4; ++i) {
      inputs[i].driveStatorCurrent = simModules[i].getDriveMotorStatorCurrent().in(Amps);
      inputs[i].driveSupplyCurrent = simModules[i].getDriveMotorSupplyCurrent().in(Amps);
      inputs[i].driveVoltsApplied = simModules[i].getDriveMotorAppliedVoltage().in(Volts);

      inputs[i].steerStatorCurrent = simModules[i].getSteerMotorStatorCurrent().in(Amps);
      inputs[i].steerSupplyCurrent = simModules[i].getSteerMotorSupplyCurrent().in(Amps);
      inputs[i].steerVoltsApplied = simModules[i].getSteerMotorAppliedVoltage().in(Volts);
    }
  }

  @Override
  public Pose2d getSimPose() {
    return mapleDriveSim.getSimulatedDriveTrainPose();
  }

  @Override
  public void updateSim() {
    this.updateSimState(
        SimulatedArena.getSimulationDt().in(Seconds),
        SimulatedBattery.getBatteryVoltage().in(Volts));
    pigeonSim.setRawYaw(mapleDriveSim.getSimulatedDriveTrainPose().getRotation().getMeasure());
    pigeonSim.setAngularVelocityZ(
        mapleDriveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative().omegaRadiansPerSecond);
  }
}
