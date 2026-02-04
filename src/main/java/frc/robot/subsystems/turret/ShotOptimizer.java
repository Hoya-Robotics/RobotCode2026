package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.FieldConstants;
import frc.robot.RobotConfig.ShooterConstants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlip;
import org.littletonrobotics.junction.Logger;

public class ShotOptimizer {
  public record OptimalShot(Rotation2d turretYaw, Rotation2d turretPitch, double turretVel) {}

  private static double G = 9.80665;

  public static OptimalShot apply() {
    var localEstimate = RobotState.getInstance().getHubLocalizedRobotPose();
    var robot = RobotState.getInstance().getOdometryPose();
    if (localEstimate != null) {
      robot = localEstimate.toPose2d();
    }
    var v_robot = RobotState.getInstance().getChassisVelocity();
    var v_robot_field = RobotState.getInstance().getFieldVelocity();

    // 1. Project robot into future (latency comp)
    var future =
        robot.exp(
            new Twist2d(
                v_robot.vxMetersPerSecond * ShooterConstants.lookaheadSeconds,
                v_robot.vyMetersPerSecond * ShooterConstants.lookaheadSeconds,
                v_robot.omegaRadiansPerSecond * ShooterConstants.lookaheadSeconds));
    var turretPos = new Pose3d(future).transformBy(ShooterConstants.robotToTurret);
    var target = AllianceFlip.apply(FieldConstants.Hub.topCenterPoint);

    // 2. Calculate field relative direction vector
    Translation3d trajectoryVector = target.minus(turretPos.getTranslation());

    // 3. Calculate field relative turret velocity
    // 	v_t = v_r(field) + w_r(field) x r_t(field)
    var turret_radius_perp =
        new Translation2d(
                -ShooterConstants.robotToTurret.getY(), ShooterConstants.robotToTurret.getX())
            .rotateBy(future.getRotation());
    var v_turret_field =
        new Translation2d(v_robot_field.vxMetersPerSecond, v_robot_field.vyMetersPerSecond)
            .plus(turret_radius_perp.times(v_robot_field.omegaRadiansPerSecond));

    double funnelHorizontalDistance =
        AllianceFlip.apply(FieldConstants.Hub.nearFace)
            .getTranslation()
            .getDistance(future.getTranslation());

    // NOTE: From here on object creation is minimized for sampling performance
    double targetX = trajectoryVector.getX();
    double targetY = trajectoryVector.getY();
    double targetZ = trajectoryVector.getZ();

    double v_turret_x = v_turret_field.getX();
    double v_turret_y = v_turret_field.getY();
    double v_turret_z = 0.0;

    double t_min = 0.0;
    double t_max = 6.0;
    int samples = 100;

    double optimalPitch = ShooterConstants.optimalPitch.in(Radians);
    double bestCost = Double.MAX_VALUE;
    var shot = new OptimalShot(Rotation2d.kZero, Rotation2d.kZero, 0.0);
    double clearance = ShooterConstants.hubFunnelClearance.in(Meters);

    // 4. Samples possible time of flight values, minimizes cost function:
    //
    // C(t) = w1 * v(t)^2 + w2 * (pitch(t) - optimal_pitch)^2 + w3 * t
    //
    // Prioritizes:
    // 	- low speed
    // 	- close to mid angle
    // 	- quick tof
    //
    // * Also obeys for funnel constraint by computing shot height at hub front-face
    //  horizontal displacement with chosen parameters and comparing to tunable clearance value
    //
    for (int i = 0; i < samples; i++) {
      double t = t_min + i * (t_max - t_min) / (samples - 1);

      double v_field_x = targetX / t;
      double v_field_y = targetY / t;
      double v_field_z = (targetZ + 0.5 * G * t * t) / t;

      double v_rel_x = v_field_x - v_turret_x;
      double v_rel_y = v_field_y - v_turret_y;
      double v_rel_z = v_field_z - v_turret_z;

      double v_hypot_xy = Math.hypot(v_rel_x, v_rel_y);
      double pitch_rads = Math.atan2(v_rel_z, v_hypot_xy);
      double v = Math.hypot(v_rel_z, v_hypot_xy);

      double vCost = v * v;
      double pitchCost = Math.pow(pitch_rads - optimalPitch, 2);
      double tCost = t;

      double cost =
          (vCost * ShooterConstants.trajectoryWeights.get(0, 0))
              + (pitchCost * ShooterConstants.trajectoryWeights.get(1, 0))
              + (tCost * ShooterConstants.trajectoryWeights.get(2, 0));

      double t_funnel = funnelHorizontalDistance / Math.hypot(v_field_x, v_field_y);
      double y_funnel = v_field_z * t_funnel - 0.5 * G * t_funnel * t_funnel;

      if (y_funnel > clearance && cost < bestCost) {
        bestCost = cost;
        shot = new OptimalShot(new Rotation2d(v_rel_x, v_rel_y), new Rotation2d(pitch_rads), v);
      }
    }

    Logger.recordOutput("OptimalShot/velocity", shot.turretVel());
    Logger.recordOutput("OptimalShot/pitch", shot.turretPitch());
    Logger.recordOutput("OptimalShot/yaw", shot.turretYaw());
    Logger.recordOutput("OptimalShot/clearanceInches", clearance);

    return shot;
  }
}
