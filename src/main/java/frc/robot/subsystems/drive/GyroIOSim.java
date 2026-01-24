package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation sim;

  public GyroIOSim(GyroSimulation sim) {
    this.sim = sim;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = true;
    inputs.yaw = sim.getGyroReading();
    inputs.yawVelocity = sim.getMeasuredAngularVelocity();
  }
}
