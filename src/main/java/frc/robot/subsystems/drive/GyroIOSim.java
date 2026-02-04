package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation sim;

  public GyroIOSim(GyroSimulation sim) {
    this.sim = sim;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yaw = sim.getGyroReading();
    inputs.yawRadPs = sim.getMeasuredAngularVelocity().in(RadiansPerSecond);
  }

  @Override
  public void setYaw(Rotation2d newYaw) {
    sim.setRotation(newYaw);
  }
}
