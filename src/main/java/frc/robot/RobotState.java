package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfig.CameraConfig;
import frc.robot.RobotConfig.OperationMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOInputsAutoLogged;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FuelSim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class RobotState {
  private FuelSim fuelSim;
  private int fuelInHopper = 0;

  private Supplier<Pose2d> simulatedDrivePoseSupplier = () -> Pose2d.kZero;
  private Drive drive;
  private Vision vision;
  private DriveIOInputsAutoLogged driveInputs;
  private VisionSystemSim visionSim;

  private static RobotState instance;

  private RobotState() {
    AutoLogOutputManager.addObject(this);
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public VisionSystemSim getVisionSim() {
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilLayout);
    }
    return visionSim;
  }

  public void registerDrivetrain(Drive drive) {
    this.drive = drive;
  }

  public void registerVision(Vision vision) {
    this.vision = vision;
  }

  public Vision getVision() {
    return vision;
  }

  public void registerFuelSim(FuelSim fuelSim) {
    this.fuelSim = fuelSim;
  }

  public void registerSimPoseSupplier(Supplier<Pose2d> supplier) {
    simulatedDrivePoseSupplier = supplier;
  }

  public void setDriveSOTM(boolean enabled) {
    drive.setSOTMEnabled(enabled);
  }

  public void resetOdometry(Pose2d pose) {
    if (drive != null) {
      drive.resetOdometry(pose);
    }
  }

  public void addVisionMeasurement(VisionObservation estimate) {
    boolean autoNeutral =
        DriverStation.isAutonomousEnabled() && FieldConstants.inNeutralZone(getEstimatedPose());
    Logger.recordOutput("RobotState/autoNeutral", autoNeutral);
    if (drive != null && !autoNeutral && RobotConfig.getMode() != OperationMode.SIM) {
      drive.addVisionMeasurement(estimate);
    }
  }

  public void addDriveInputs(DriveIOInputsAutoLogged inputs) {
    this.driveInputs = inputs;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(driveInputs.Speeds, driveInputs.gyroYaw);
  }

  public ChassisSpeeds getRobotVelocity() {
    return driveInputs.Speeds;
  }

  public FuelSim getFuelSim() {
    return fuelSim;
  }

  public void addFuel() {
    fuelInHopper += 1;
    Logger.recordOutput("RobotState/simFuelCount", fuelInHopper);
  }

  public boolean consumeFuel() {
    boolean hasFuel = fuelInHopper > 0;
    if (hasFuel) fuelInHopper -= 1;
    Logger.recordOutput("RobotState/simFuelCount", fuelInHopper);
    return hasFuel;
  }

  @AutoLogOutput(key = "RobotState/estimatedPose")
  public Pose2d getEstimatedPose() {
    if (driveInputs == null) {
      return Pose2d.kZero;
    }
    return new Pose2d(driveInputs.Pose.getTranslation(), driveInputs.gyroYaw);
  }

  public Pose2d getSimulatedPose() {
    return simulatedDrivePoseSupplier.get();
  }

  public record VisionObservation(
      CameraConfig source, Pose2d pose, Vector<N3> stdDevs, Time timestamp) {
    public double timestampSeconds() {
      return timestamp.in(Seconds);
    }
  }
}
