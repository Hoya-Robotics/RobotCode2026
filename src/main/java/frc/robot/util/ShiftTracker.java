package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class ShiftTracker {
  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
  private static Timer shiftTimer = new Timer();

  public static void initialize() {
    shiftTimer.restart();
  }

  public static double shiftTimeRemaining() {
    if (DriverStation.isAutonomous()) return Float.POSITIVE_INFINITY;
    double t = shiftTimer.get();
    int shift = 0;
    for (int i = 0; i < shiftStartTimes.length; ++i) {
      if (t > shiftStartTimes[i]) {
        shift = i;
        break;
      }
    }

    double remaining = (shiftEndTimes[shift] - shiftStartTimes[shift]) - t - shiftStartTimes[shift];

		Logger.recordOutput("ShiftTracker/currentShift", shift);
		Logger.recordOutput("ShiftTracker/teleopTime", t);
		Logger.recordOutput("ShiftTracker/shiftTimeReamining", remaining);
		return remaining;
  }
}
