package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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

  private static GameShift currentShift = GameShift.Transition;
  private static boolean activeFirst = false;
  private static Timer teleopTimer = new Timer();

  public static boolean isHubActive() {
    return switch (currentShift) {
      case Transition -> true;
      case Endgame -> true;
      default -> activeFirst == ((currentShift.ordinal() - 1) % 2 == 0);
    };
  }

  public static double timeTillShiftEnds() {
    double t = teleopTimer.get();
    return Math.max(
        0.0,
        switch (currentShift) {
          case Transition -> 10 - t;
          case Endgame -> 140 - t;
          default -> (10 + 25 * currentShift.ordinal()) - t;
        });
  }

  public static void start() {
    teleopTimer.start();
  }

  public static void run() {
    String fmsMessage = DriverStation.getGameSpecificMessage();
    boolean redInactiveFirst = false;
    // if (fmsMessage.isEmpty()) return;
    if (fmsMessage.isEmpty()) {
      redInactiveFirst = true;
    } else {
      switch (fmsMessage.charAt(0)) {
        case 'R' -> redInactiveFirst = true;
        case 'B' -> redInactiveFirst = false;
        default -> redInactiveFirst = true;
      }
    }
    activeFirst = FieldConstants.isBlueAlliance() == redInactiveFirst;

    double teleopTime = teleopTimer.get();
    if (teleopTime <= 10) {
      currentShift = GameShift.Transition;
    } else if (teleopTime >= 110) {
      currentShift = GameShift.Endgame;
    } else {
      int n = (int) ((teleopTime - 10) / 25);
      currentShift = GameShift.values()[n + 1];
    }
  }
}
