package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.*;
import frc.robot.subsystems.drive.*;
import java.util.NoSuchElementException;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.photonvision.PhotonUtils;

public class RobotState {
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(RobotConfig.DriveConstants.moduleTranslations);
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final TimeInterpolatableBuffer<Pose2d> globalPoseBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);
  private Pose2d estimatedPose = Pose2d.kZero;
  private Pose2d odometryPose = Pose2d.kZero;
  private ChassisSpeeds odometrySpeeds = new ChassisSpeeds();
  private Pose3d robotToHub;
  private Supplier<Pose2d> simulatedDrivePoseSupplier;
  private final Matrix<N3, N1> odometryStdDevs = VecBuilder.fill(0.003, 0.003, 0.002);
  private final Matrix<N3, N1> processVariance = odometryStdDevs.elementPower(2);

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {
    AutoLogOutputManager.addObject(this);
  }

  public void addSimPoseSupplier(Supplier<Pose2d> supplier) {
    simulatedDrivePoseSupplier = supplier;
  }

  public void hardSetOdometry(Pose2d pose) {
    this.odometryPose = pose;
  }

  public void hardSetKalmanPose(Pose2d pose) {
    this.globalPoseBuffer.clear();
    this.estimatedPose = pose;
  }

  public ChassisSpeeds getChassisVelocity() {
    return odometrySpeeds;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(odometrySpeeds, odometryPose.getRotation());
  }

  @AutoLogOutput(key = "RobotState/hubLocalizedPose")
  public Pose3d getHubLocalizedRobotPose() {
    return robotToHub;
  }

  @AutoLogOutput(key = "RobotState/odometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  @AutoLogOutput(key = "RobotState/estimatedPose")
  public Pose2d getEstimatedRobotPose() {
    return estimatedPose;
  }

  public Pose2d getSimulatedDrivePose() {
    return simulatedDrivePoseSupplier.get();
  }

  public void addOdometryObservation(OdometryObservation observation) {
    var twist = kinematics.toTwist2d(lastModulePositions, observation.modulePositions());
    lastModulePositions = observation.modulePositions();

    odometrySpeeds = observation.speeds();
    var lastOdometryPose = odometryPose;

    odometryPose = new Pose2d(odometryPose.exp(twist).getTranslation(), observation.gyroYaw());
    globalPoseBuffer.addSample(observation.timestamp(), odometryPose);

    // Account for gyro twist
    estimatedPose = estimatedPose.exp(lastOdometryPose.log(odometryPose));
  }

  // Solved closed gain kalman measurement step with vision observation
  public void addVisionObservation(VisionFieldPoseEstimate visionEstimate) {
    try {
      if (globalPoseBuffer.getInternalBuffer().lastKey() - 2.0 > visionEstimate.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    var sample = globalPoseBuffer.getSample(visionEstimate.timestamp());
    if (sample.isEmpty()) {
      return;
    }

    var sampleToOdometry = new Transform2d(sample.get(), odometryPose);

    Pose2d estimateAtTime = estimatedPose.plus(sampleToOdometry.inverse());
    var measurementVariance = visionEstimate.stdDevs().elementPower(2);

    // We perform a kalman filter update step as a static state estimation
    // A=0, B=0, D=0, C=I
    // Kalman gain derived here:
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/algorithms.md
    // K = q/(q + sqrt(qr))
    // Where:
    // 	q = process noise (covariance matrix)
    // 	r = measurement noise (covariance matrix)
    //
    //  x ~ p_1(0, odometryVariance)
    //  y ~ p_2(0, visionVariance)
    //
    Matrix<N3, N3> K = new Matrix<>(Nat.N3(), Nat.N3());
    for (int i = 0; i < 3; ++i) {
      double q = processVariance.get(i, 0);
      double r = measurementVariance.get(i, 0);
      K.set(i, i, q / (q + Math.sqrt(q * r)));
    }

    var observedTransformation = new Transform2d(estimateAtTime, visionEstimate.pose());
    Matrix<N3, N1> transformationMatrix =
        VecBuilder.fill(
            observedTransformation.getX(),
            observedTransformation.getY(),
            observedTransformation.getRotation().getRadians());
    var kalmanTransformMatrix = K.times(transformationMatrix);
    var kalmanTransform =
        new Transform2d(
            kalmanTransformMatrix.get(0, 0),
            kalmanTransformMatrix.get(1, 0),
            new Rotation2d(kalmanTransformMatrix.get(2, 0)));

    estimatedPose = estimateAtTime.transformBy(kalmanTransform);
  }

  public void addHubObservation(HubObservation observation) {
    FieldConstants.aprilLayout
        .getTagPose(observation.tid)
        .ifPresent(
            (fieldRelTagPose) -> {
              robotToHub =
                  PhotonUtils.estimateFieldToRobotAprilTag(
                      observation.cameraToTarget(),
                      fieldRelTagPose,
                      observation.robotToCamera().inverse());
            });
  }

  // Kalman probably unecessary for hub? Just use latest?
  // Maybe reject using an ambiguity threshold
  public record HubObservation(Transform3d robotToCamera, Transform3d cameraToTarget, int tid) {}

  public record VisionFieldPoseEstimate(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  public record OdometryObservation(
      ChassisSpeeds speeds,
      SwerveModulePosition[] modulePositions,
      Rotation2d gyroYaw,
      double timestamp) {}
}
