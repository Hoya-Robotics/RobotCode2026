package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotConfig;
import frc.robot.util.FuelSim;
import frc.robot.util.StateSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

enum TurretState {
  SHOOT,
  HOLD
}

public class PlaceholderTurret extends StateSubsystem<TurretState> {
  private final Transform3d robotToTurret = new Transform3d();
  private int fuelRemaining = 8;
  private Rotation2d turretYaw;
  private BooleanSupplier fuelLoaded;

  public PlaceholderTurret() {
    setState(TurretState.SHOOT);
  }

  @Override
  public void periodic() {
    statePeriodic();
  }

  public void letShoot() {
    setState(TurretState.SHOOT);
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case SHOOT:
        if (fuelRemaining <= 0) {
          setState(TurretState.HOLD);
          break;
        }
        var shot = ShotOptimizer.apply();
        turretYaw = shot.turretYaw();
        FuelSim.getInstance()
            .launchFuel(
                MetersPerSecond.of(shot.turretVel()),
                shot.turretPitch().getMeasure(),
                shot.turretYaw().getMeasure(),
                RobotConfig.robotToTurret.getMeasureZ());
        fuelRemaining -= 1;
        Logger.recordOutput("Turret/fuelRemaining", fuelRemaining);
        setState(TurretState.HOLD);
        break;
      default:
        break;
    }
  }
}
