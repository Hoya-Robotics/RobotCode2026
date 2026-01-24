package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

public class VisionIOLimelight implements VisionIO {
  // Apriltag positions
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;

  private final IntegerSubscriber tidSubscriber;

  // Latency
  private final DoubleSubscriber tlSubscriber;
  private final DoubleSubscriber clSubscriber;

  private final DoubleArraySubscriber megatagSubscriber;
  private final DoubleArraySubscriber rawFudicialSubscriber;

  private final IntegerPublisher throttlePublisher;
  private boolean throttling;
  private final int index;

  public VisionIOLimelight(int index) {
    this.index = index;

    var table = NetworkTableInstance.getDefault().getTable(VisionConstants.cameraNames[index]);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    tidSubscriber = table.getIntegerTopic("tid").subscribe(0);
    tlSubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    clSubscriber = table.getDoubleTopic("cl").subscribe(0.0);
    throttlePublisher = table.getIntegerTopic("throttle_set").publish();
    throttlePublisher.set(200);
    throttling = true;

    megatagSubscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    rawFudicialSubscriber = table.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if ((DriverStation.isEnabled() || DriverStation.isDSAttached()) && throttling) {
      throttling = false;
      throttlePublisher.set(0);
    }

    double bestTx = txSubscriber.get();
    double bestTy = tySubscriber.get();
    int bestTid = (int) tidSubscriber.get();

    final double latency = (tlSubscriber.get() + clSubscriber.get()) / 1e3;
    double timestamp = Timer.getFPGATimestamp() - latency;

    var allTargets = new ArrayList<AprilFiducial>();
    var fudicialsArray = rawFudicialSubscriber.get();
    for (int i = 0; i < fudicialsArray.length / 7; ++i) {
      int tid = (int) fudicialsArray[(i * 7) + 0];
      double tx = fudicialsArray[(i * 7) + 1];
      double ty = fudicialsArray[(i * 7) + 2];
      double ambig = fudicialsArray[(i * 7) + 6];

      allTargets.add(new AprilFiducial(this.index, tx, ty, tid, timestamp, ambig));
    }

    // Reset gyro yaw with megatag estimate
    // TODO: filter this so we dont reset every pass

    inputs.allTargets = (AprilFiducial[]) allTargets.toArray();
    inputs.bestTarget = new AprilFiducial(this.index, bestTx, bestTy, bestTid, timestamp, 0.0);
    inputs.megatagYaw = Degrees.of(megatagSubscriber.get()[5]);
    inputs.update_yaw = false;
  }
}
