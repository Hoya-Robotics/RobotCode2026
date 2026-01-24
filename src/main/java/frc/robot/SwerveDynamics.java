package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Module;
import org.ejml.simple.SimpleMatrix;

/*
 * Custom swerve drive kinematics and odometry math implementation
 *
 * Advantages over WPIlib are primarily for greater understanding,
 * and customizability
 *
 * Inspired by Orbit 1690
 */
public class SwerveDynamics {
  public static class ChassisVelocity {
    public AngularVelocity omega = RadiansPerSecond.zero();
    public Translation2d velocityVector = Translation2d.kZero;

    // Precompute pseudo inverse chasiss matrix to reduce per cycle allocations
    private static final SimpleMatrix pseudoMatrix =
        new SimpleMatrix(
                new double[][] {
                  // Cols = [ vx, vy, omega ]
                  {1, 0, -DriveConstants.modulePositions[0].getY()},
                  {0, 1, DriveConstants.modulePositions[0].getX()},
                  {1, 0, -DriveConstants.modulePositions[1].getY()},
                  {0, 1, DriveConstants.modulePositions[1].getX()},
                  {1, 0, -DriveConstants.modulePositions[2].getY()},
                  {0, 1, DriveConstants.modulePositions[2].getX()},
                  {1, 0, -DriveConstants.modulePositions[3].getY()},
                  {0, 1, DriveConstants.modulePositions[3].getX()},
                })
            .pseudoInverse();

    public ChassisVelocity() {}

    public ChassisVelocity(AngularVelocity omega, Translation2d v) {
      this.omega = omega;
      this.velocityVector = v;
    }

    public static ChassisVelocity fromFieldRelative(
        double vx, double vy, double vw, Rotation2d robotYaw) {
      var v = new Translation2d(vx, vy).rotateBy(robotYaw.unaryMinus());
      return new ChassisVelocity(RadiansPerSecond.of(vw), v);
    }

    /* Forward kinematecs
     * Uses least squares method to solve unknown vector
     * https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Linear_least-squares
     *
     * Solves Ax = b
     * where:
     * 	A = constraint matrix  (8, 3)
     * 	b = module velocity vector  (8, 1)
     *  x = chassis velocity vector (3, 1)
     */
    public static ChassisVelocity forwardKinematics(Module[] modules) {
      SimpleMatrix moduleVelocityMatrix = new SimpleMatrix(8, 1);
      for (int i = 0; i < 4; ++i) {
        final Translation2d vel = modules[i].getVelocity();
        moduleVelocityMatrix.setColumn(0, i * 2, new double[] {vel.getX(), vel.getY()});
      }

      // x ~= A+ @ b
      final var bestFit = pseudoMatrix.mult(moduleVelocityMatrix);
      return new ChassisVelocity(
          RadiansPerSecond.of(bestFit.get(2)), new Translation2d(bestFit.get(0), bestFit.get(1)));
    }

    /*
     * Inverse kinematics to solve for individual module velocities
     * from chasis velocities
     */
    public ModuleVelocity[] inverseKinematics(Module[] modules) {
      var mvs = new ModuleVelocity[4];
      double mxv = 0.0;

      for (int i = 0; i < 4; ++i) {
        final Translation2d radius = modules[i].getChassisPosition();
        double omegaRad = omega.in(RadiansPerSecond);
        // v = w x r
        Translation2d rotationVector =
            new Translation2d(omegaRad * -radius.getY(), omegaRad * radius.getX());
        mvs[i] = new ModuleVelocity(this.velocityVector.plus(rotationVector));
        mxv = Math.max(mxv, mvs[i].magnitude().in(MetersPerSecond));
      }

      // Normalize module velocities to preserve direction when exceeding speed limits
      final double factor = DriveConstants.maxLinearSpeed / mxv;
      if (mxv > DriveConstants.maxLinearSpeed && mxv > 1e-6) {
        for (int i = 0; i < 4; ++i) mvs[i] = mvs[i].scale(factor);
      }
      return mvs;
    }
  }

  public static class ModuleVelocity {
    // How to represent magntiude vs direction?
    private LinearVelocity magnitude;
    private Rotation2d heading;

    public ModuleVelocity(double magnitudeMeters, Rotation2d heading) {
      this.magnitude = MetersPerSecond.of(magnitudeMeters);
      this.heading = heading;
    }

    public Translation2d toTranslation2d() {
      return new Translation2d(magnitude.in(MetersPerSecond), heading);
    }

    public ModuleVelocity(Translation2d vector) {
      this.magnitude = MetersPerSecond.of(vector.getNorm());
      this.heading = vector.getAngle();
    }

    public ModuleVelocity scale(double scalar) {
      this.magnitude = magnitude.times(scalar);
      return this;
    }

    public final LinearVelocity magnitude() {
      return this.magnitude;
    }

    public final Rotation2d getHeading() {
      return this.heading;
    }

    public void optimize(Rotation2d curHeading) {
      Rotation2d error =
          Rotation2d.fromRadians(
              MathUtil.angleModulus(this.heading.minus(curHeading).getRadians()));
      // Prevents uneccessary heading changes > 90°, takes opposite angle
      if (Math.abs(error.getRadians()) > Math.PI / 2) {
        magnitude = magnitude.times(-1);
        heading = heading.plus(Rotation2d.k180deg);
        error = error.plus(Rotation2d.k180deg);
      }
      // Reduces speed opposite to rotation vector
      magnitude = magnitude.times(error.getCos());
    }
  }

  /* Improves odometry accuracy by accounting for circular movement
   * in module displacements
   *
   * returns a module relative arc displacement vector
   *
   * Chief delphi thread: https://www.chiefdelphi.com/t/orbit-1690-2025-robot-reveal-whisper/492064/150?u=raine1
   */
  public static Translation2d getArcDisplacement(
      Translation2d chassisPos,
      Distance lastPos,
      Distance curPos,
      Rotation2d lastHeading,
      Rotation2d curHeading) {
    final Distance linearDist = curPos.minus(lastPos);
    final Rotation2d theta = curHeading.minus(lastHeading);
    final Rotation2d moduleToChassis = lastHeading;

    // Handle small theta values, else radius calculation will
    // blow up, just return linear displacement
    if (Math.abs(theta.getRadians()) < 1e-6) {
      return new Translation2d(linearDist.in(Meters), moduleToChassis);
    }

    // r = a / theta
    final double radius = linearDist.in(Meters) / theta.getRadians();
    final Translation2d centerToLast =
        new Translation2d(radius, lastHeading.minus(Rotation2d.fromDegrees(90)));
    final Translation2d centerToCur = centerToLast.rotateBy(theta);

    return centerToCur.minus(centerToLast);
  }
}
