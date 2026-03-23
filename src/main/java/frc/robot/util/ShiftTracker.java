package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class ShiftTracker {
  public static enum GameShift {
    Transition,
    Shift1,
    Shift2,
    Shift3,
    Shift4,
    Endgame
  }

  private static final int[] matchShiftTimes = new int[] {105, 80, 55, 30};
  private static GameShift currentShift = GameShift.Transition;
  private static boolean activeFirst = false;

  public static boolean isHubActive() {
    return switch (currentShift) {
      case Transition -> true;
      case Endgame -> true;
      default -> activeFirst == ((currentShift.ordinal() - 1) % 2 == 0);
    };
  }

  public static double timeTillHubActive() {
    if (isHubActive()) return -1.0;
    double timeRemainingNext = matchShiftTimes[currentShift.ordinal()];
    return DriverStation.getMatchTime() - timeRemainingNext;
  }

  public static double timeTillShiftEnds() {
    double mt = DriverStation.getMatchTime();
    return switch (currentShift) {
      case Transition -> mt - 130;
      case Endgame -> mt;
      default -> mt - matchShiftTimes[currentShift.ordinal() - 1];
    };
  }

  public static void run() {
    String fmsMessage = DriverStation.getGameSpecificMessage();
    boolean redInactiveFirst = false;
		if (fmsMessage.isEmpty()) return;
    switch (fmsMessage.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {}
    }
    activeFirst = FieldConstants.isBlueAlliance() == redInactiveFirst;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime > 130) {
      currentShift = GameShift.Transition;
    } else if (matchTime <= 30.0) {
      currentShift = GameShift.Endgame;
    } else {
      for (int i = 0; i < matchShiftTimes.length; ++i) {
        if (matchTime > matchShiftTimes[i]) {
          currentShift = GameShift.values()[i + 1];
          break;
        }
      }
    }
  }
}
