package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotConfig;
import frc.robot.RobotState;
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

  private Translation3d launchVector(double vel, Rotation2d pitch, Rotation2d yaw) {
    var robotSpeeds = RobotState.getInstance().getFieldVelocity();

    double horizontal = vel * pitch.getCos();
    double vx = horizontal * yaw.getCos() + robotSpeeds.vxMetersPerSecond * yaw.getCos();
    double vy = horizontal * yaw.getSin() + robotSpeeds.vyMetersPerSecond * yaw.getSin();
    double vz = vel * pitch.getSin();

    return new Translation3d(vx, vy, vz);
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
        var shot = RobotState.getInstance().getOptimalShot();
        turretYaw = shot.turretYaw();
        var shotVector = launchVector(shot.turretVel(), shot.turretPitch(), shot.turretYaw());
        FuelSim.getInstance()
            .spawnFuel(
                new Pose3d(RobotState.getInstance().getOdometryPose())
                    .transformBy(RobotConfig.robotToTurret)
                    .getTranslation(),
                shotVector);
        fuelRemaining -= 1;
        Logger.recordOutput("Turret/fuelRemaining", fuelRemaining);
        Logger.recordOutput("Turret/shotVector", shotVector);
        setState(TurretState.HOLD);
        break;
      default:
        break;
    }
  }
}
