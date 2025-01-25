// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: DrivetrainSubsystem.java
// Intent: Forms the prelminary code for drive train subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// TODO - START HERE!! need to merge this with CommandSwerveDrivetrain. 
// map existing functionality into available swerve actions from list here: 
// https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/swerve/SwerveRequest.html

package frc.robot.subsystems;

import java.util.*;
import java.lang.Math;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.config.RobotConfig;

import static frc.robot.control.Constants.*;

import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.Telemetry;
import frc.robot.common.DrivetrainSwerveConfig;
import frc.robot.control.SwerveDriveMode;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.control.SubsystemCollection;
import frc.robot.common.MotorUtils;
import frc.robot.common.VisionMeasurement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.MatBuilder;

public class DrivetrainSubsystem extends SubsystemBase {
  private CameraSubsystem cameraSubsystem;

  private boolean useVision = false;

  StructArrayPublisher<SwerveModuleState> publisher;

  private final double deltaTimeSeconds = 0.02; // 20ms scheduler time tick

  private static final DrivetrainSwerveConfig swerveConfig = 
  InstalledHardware.tedDrivetrainInstalled ? Constants.tedDrivertainConfig : Constants.fooDrivetrainConfig;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.SWERVE_MAX_SPEED;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 100.0;
  public static final double MAX_DECELERATION_METERS_PER_SECOND_SQUARED = 100.0;

  private final Telemetry logger = new Telemetry(MAX_VELOCITY_METERS_PER_SECOND);
    /**
   * The maximum angular velocity of the robot in radians per second.
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.SWERVE_MAX_SPEED /
          Math.hypot(swerveConfig.getTrackwidthMeters() / 2.0, swerveConfig.getWheelbaseMeters() / 2.0);
  public static final double MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.06; // 0.06 a magic number based on testing
  private double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 70.0;

  private static RobotConfig pathPlannerRobotConfig; 

  // Standard deviations for poseEstimator updates
  // The wpilib matrix constructor requires sizes specified as Nat types. 
  // Determined these settings from looking at other team's settings
  // we use a high variance for the camera yaw because we don't want it to override 
  // the odometry yaw, which comes from the very accurate IMU
  private Matrix<N3,N1> visionStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), new double[]{0.7, 0.7, 10});
  private Matrix<N3,N1> odometryStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), new double[]{0.1, 0.1, 0.1});

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private ChassisSpeeds previousChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private double speedReductionFactor = 1.0;

  private SwerveDriveMode swerveDriveMode = SwerveDriveMode.NORMAL_DRIVING;

  private TunerSwerveDrivetrain drivetrain = new TunerSwerveDrivetrain(TunerConstants.DrivetrainConstants, 0, odometryStdDev, visionStdDev,
                                                                       TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

  private SwerveRequest.FieldCentric fieldCentricDriveController = new SwerveRequest.FieldCentric().withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  private SwerveRequest.SwerveDriveBrake brakeDriveController = new SwerveRequest.SwerveDriveBrake();

  /**
   * Constructor for this DrivetrainSubsystem
   */
  public DrivetrainSubsystem(SubsystemCollection subsystems) {
    if(InstalledHardware.limelightInstalled){
      cameraSubsystem = subsystems.getCameraSubsystem();
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // get Path Planner config from GUI settings.
    try{
      pathPlannerRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  /**
   * Method avaliable so that callers can update the chassis speeds to induce changes in robot movement
   * @param updatedChassisSpeeds - the updated chassis speeds (x, y and rotation)
   */
  public void drive(ChassisSpeeds updatedChassisSpeeds) {
    this.chassisSpeeds = updatedChassisSpeeds;
  }
  
  /**
   * returns chassis speeds (robot relative)
   * @return chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds(){
    return chassisSpeeds;
  }

  /**
   * Method to get the current speed reduction factor
   * @return - the current speed reduction factor
   */
  public double getSpeedReductionFactor() {
    return this.speedReductionFactor;
  }
  
  public RobotConfig getPathPlannerConfig() {
    return pathPlannerRobotConfig;
  }

  /**
   * a method that makes the odometry update with vision updates
   * @param shouldUseVision true to use vision, false to not
   */
  public void setUseVision(boolean shouldUseVision){
    this.useVision = shouldUseVision;
  }
  
  /**
   * Method to set the current speed reduction factor to a new value
   * @param value - the new speed reduction factor
   */
  public void setSpeedReductionFactor(double value) {
    this.speedReductionFactor = MotorUtils.truncateValue(value, 0.0, 1.0);
  }

  /**
   * Obtains the current gyroscope's rotation about the Z axis when looking down at the robot where positive is measured in the 
   * counter-clockwise direction.
   * 
   * @return A Rotation2d that describes the current orentation of the robot.
   */
  public Rotation2d getGyroscopeRotation() {
    return new Rotation2d(0.0);
    //return drivetrain.getPigeon2().getRotation2d();
  }

  /**
   * A method to get the current position of the robot
   * @return the current Pose2d position of the robot
   */
  public Pose2d getRobotPosition()
  {
    return drivetrain.getState().Pose;
  }
 
   /**
    * A method to obtain the swerve drive mode
    * @return the mode
    */
   public SwerveDriveMode getSwerveDriveMode() {
    return swerveDriveMode;
  }

  /**
   * Perodic for this subsystem - very important for it to run every scheduler cycle - 50Hz
   */
  @Override
  public void periodic() {
    // update robot position with vision
    if(InstalledHardware.limelightInstalled){
      this.addVisionMeasurement(cameraSubsystem.getVisionBotPose());
    } 

    SmartDashboard.putNumber("Gyro Yaw", drivetrain.getPigeon2().getYaw().getValueAsDouble());
    this.displayDiagnostics();

    // TODO update this with calls to CommandSwerveDrivetrain
    // have to figure out Inheritance structure. Think this needs to extend CommandSwerveDrivetrain
    if (swerveDriveMode == SwerveDriveMode.IMMOVABLE_STANCE && chassisSpeedsAreZero()) {
      // only change to ImmovableStance if chassis is not moving.
      // otherwise, we could tip the robot moving to this stance when bot is at high velocity
      drivetrain.setControl(brakeDriveController);
    }
    else { // SwerveDriveMode.NORMAL_DRIVING
      // apply the speed reduction factor to the chassis speeds
      ChassisSpeeds reducedChassisSpeeds = new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond * this.speedReductionFactor, 
        chassisSpeeds.vyMetersPerSecond * this.speedReductionFactor, 
        // different speed reduction factor for rotation
        chassisSpeeds.omegaRadiansPerSecond * Math.min(1.0, this.speedReductionFactor * 1.25));

      // apply acceleration control
      reducedChassisSpeeds = limitChassisSpeedsAccel(reducedChassisSpeeds);
      previousChassisSpeeds = reducedChassisSpeeds; 

      // take the current 'requested' chassis speeds and ask the ask the swerve modules to attempt this
      // first we build a theoretical set of individual module states that the chassisSpeeds would corespond to
      //states = swerveKinematics.toSwerveModuleStates(reducedChassisSpeeds);
      drivetrain.setControl(fieldCentricDriveController
                            .withVelocityX(reducedChassisSpeeds.vxMetersPerSecond)
                            .withVelocityY(reducedChassisSpeeds.vyMetersPerSecond)
                            .withRotationalRate(reducedChassisSpeeds.omegaRadiansPerSecond));
    } 
  }

  /**
   * A method to set the current position of the robot
   * @param updatedPosition - the new position of the robot
   */
  public void setRobotPosition(Pose2d updatedPosition) {
    drivetrain.resetPose(updatedPosition);
  }

  /**
   * A method to set the swerve drive mode
   * @param swerveDriveMode
   */
  public void setSwerveDriveMode(SwerveDriveMode swerveDriveMode) {
    this.swerveDriveMode = swerveDriveMode;
    // if (swerveDriveMode == SwerveDriveMode.IMMOVABLE_STANCE) DO NOT set chassis speeds to 0.0 here.
    // we need to allow the robot to decel properly so that it doesn't tip.  
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    //drivetrain.seedFieldCentric();
  }

  /**
   * A method to zero the current position
   */
  public void zeroRobotPosition() {
    this.setRobotPosition(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
  }

  /**
   * A method that updates the robot position with a vision measurement
   * @param visionMeasurement the most recent vision measurement provided by vision subsystem
   */
  private void addVisionMeasurement(VisionMeasurement visionMeasurement){
    // for now ignore all vision measurements that are null or contained robot position is null
    if (visionMeasurement != null && useVision){
      Pose2d visionComputedMeasurement = visionMeasurement.getRobotPosition();
      if(visionComputedMeasurement != null) {
        //we want to reject vision measurements that are more than 1 meter away in case vison gives a bad read
        if(visionComputedMeasurement.getTranslation().getDistance(getRobotPosition().getTranslation()) <= 1){
          drivetrain.addVisionMeasurement(visionComputedMeasurement, visionMeasurement.getTimestamp());
        }
      }
    }
}

  private boolean chassisSpeedsAreZero(){
    return (chassisSpeeds.vxMetersPerSecond == 0.0) 
    && (chassisSpeeds.vyMetersPerSecond == 0.0) 
    && (chassisSpeeds.omegaRadiansPerSecond == 0.0);
  }
  
  /**
   * Clamps the chassis speeds between a min and max.  
   * Separete min and max for translational (x,y) vs. rotational speeds
   * @param chassisSpeeds
   * @param translationMin
   * @param translationMax
   * @param rotationMin
   * @param rotationMax
   * @return clamped chassisSpeeds
   */
  private ChassisSpeeds clampChassisSpeeds(
    ChassisSpeeds chassisSpeeds, 
    double translationMin,
    double translationMax,
    double rotationMin,
    double rotationMax){
    // do not scale omega
    //double clampedOmega = MotorUtils.doubleSidedClamp(chassisSpeeds.omegaRadiansPerSecond, rotationMin, rotationMax);
    // if one or both of X or Y needs to be clamped, we need to scale both proportionally
    // form a Translation2d of x, y so the math is easier
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double velocity = translation.getNorm();
    double clampedVelocity = MotorUtils.doubleSidedClamp(velocity, translationMin, translationMax);
    // scale the translation by the clamped velocity scale factor
    double scale = clampedVelocity/velocity;
    translation = translation.times(scale);
    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Clamps the chassis speeds between the drivetrain min and max velocities
   * @param chassisSpeeds
   * @return clamped chassisSpeeds
   */
  private ChassisSpeeds clampChassisSpeeds(ChassisSpeeds chassisSpeeds){
    return this.clampChassisSpeeds(chassisSpeeds, 
    0, // TODO this used to be MIN_VELOCITY_BOUNDARY_METERS_PER_SECOND
    Constants.SWERVE_MAX_SPEED, 
    MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND, 
    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
  }

  private SwerveModuleState[] getImmovableStanceStates(){
    // set wheels in "X" pattern
    return new SwerveModuleState[] {
      // frontLeftModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
      // frontRightModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      // backLeftModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
      // backRightModule
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
    }; 
  };

  /**
   * Limits chassis speeds based on max allowable acceleration
   * @param speeds
   * @return
   */
  private ChassisSpeeds limitChassisSpeedsAccel(ChassisSpeeds speeds) {
    // translational acceleration has different accel/decel limits
    double xVelocityLimited = limitAxisSpeed(speeds.vxMetersPerSecond, previousChassisSpeeds.vxMetersPerSecond, 
    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, 
    MAX_DECELERATION_METERS_PER_SECOND_SQUARED); // acceleration reduction factor does not apply to deceleration
    double yVelocityLimited = limitAxisSpeed(speeds.vyMetersPerSecond, previousChassisSpeeds.vyMetersPerSecond, 
    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, 
    MAX_DECELERATION_METERS_PER_SECOND_SQUARED); // acceleration reduction factor does not apply to deceleration
    // angular acceleration is has same accel/decel limit
    double omegaVelocityLimited = limitAxisSpeed(speeds.omegaRadiansPerSecond, previousChassisSpeeds.omegaRadiansPerSecond, 
    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED, 
    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    return new ChassisSpeeds(xVelocityLimited, yVelocityLimited, omegaVelocityLimited);
  }

  /**
   * Limits axis speed based on max allowable acceleration
   * @param commandedSpeed
   * @param previousSpeed
   * @param maxAccel
   * @return limited speed
   */
  private double limitAxisSpeed(double commandedSpeed, double previousSpeed, double maxAccel, double maxDecel){
      double accel = (commandedSpeed - previousSpeed)/deltaTimeSeconds;
      double speedLimited = commandedSpeed;
      if (accel > maxAccel){ // positive accel is greater than max
        // new velocity is the old velocity + the maximum allowed change toward the new direction
        speedLimited = previousSpeed + Math.copySign(maxAccel * deltaTimeSeconds, accel);
      } else if (accel < -maxDecel) { // accel is less than negative max
        speedLimited = previousSpeed + Math.copySign(maxDecel * deltaTimeSeconds, accel);
      }
      return speedLimited;
  }

  private void displayDiagnostics(){
    SmartDashboard.putNumber("RobotFieldHeadingDegrees", drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("RobotFieldXCoordinateMeters", drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("RobotFieldYCoordinateMeters", drivetrain.getState().Pose.getY());

  }
}