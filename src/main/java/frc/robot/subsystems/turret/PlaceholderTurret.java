package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
    fuelLoaded =
        () -> {
          return DriverStation.isEnabled()
              && ((int) Timer.getFPGATimestamp()) % 1 == 0
              && fuelRemaining > 0;
        };
    setState(TurretState.SHOOT);
  }

  private Translation3d launchVector(double vel, Rotation2d pitch) {
    var robot = RobotState.getInstance().getOdometryPose();
    var robotSpeeds = RobotState.getInstance().getFieldVelocity();
    double vx = vel * pitch.getCos() * robot.getRotation().getCos() + robotSpeeds.vxMetersPerSecond;
    double vy = vel * pitch.getCos() * robot.getRotation().getSin() + robotSpeeds.vyMetersPerSecond;
    return new Translation3d(vx, vy, vel * pitch.getSin());
  }

  @Override
  public void periodic() {
    statePeriodic();
  }

  @Override
  public void applyState() {
    switch (getCurrentState()) {
      case SHOOT:
        if (fuelLoaded.getAsBoolean()) {
          var shot = RobotState.getInstance().getOptimalShot();
          turretYaw = shot.turretYaw();
          var shotVector = launchVector(shot.turretVel(), shot.turretPitch());
          FuelSim.getInstance()
              .spawnFuel(
                  new Pose3d(RobotState.getInstance().getOdometryPose())
                      .transformBy(RobotConfig.robotToTurret)
                      .getTranslation(),
                  shotVector);
          fuelRemaining -= 1;
          Logger.recordOutput("Turret/fuelRemaining", fuelRemaining);
          Logger.recordOutput("Turret/shotVector", shotVector);
        }
        break;
      default:
        break;
    }
  }
}
