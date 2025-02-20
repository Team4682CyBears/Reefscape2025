// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: DrivetrainSubsystem.java
// Intent: Forms the prelminary code for drive train subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import java.lang.Math;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;

import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.LimelightHelpers;
import frc.robot.Telemetry;
import frc.robot.common.DrivetrainSwerveConfig;
import frc.robot.control.SwerveDriveMode;
import frc.robot.generated.Telemetry;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.MatBuilder;

public class DrivetrainSubsystem extends SubsystemBase {
  private CameraSubsystem cameraSubsystem;

  private boolean useVision = true;

  private boolean furtherThanAMeter = false;

  private boolean displayOdometryDiagnostics = false;

  private Timer timer = new Timer();


  StructArrayPublisher<SwerveModuleState> publisher;

  private final double deltaTimeSeconds = 0.02; // 20ms scheduler time tick

  private static final DrivetrainSwerveConfig swerveConfig = InstalledHardware.tedDrivetrainInstalled
      ? Constants.tedDrivertainConfig
      : Constants.fooDrivetrainConfig;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.SWERVE_MAX_SPEED;
  // TODO change this to something reasonable. Was 6 in TED
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 100.0;
  // TODO change this to something reasonable. Was 12 in TED
  public static final double MAX_DECELERATION_METERS_PER_SECOND_SQUARED = 100.0;

  private final Telemetry logger = new Telemetry(MAX_VELOCITY_METERS_PER_SECOND);
  /**
   * The maximum angular velocity of the robot in radians per second.
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.SWERVE_MAX_SPEED /
      Math.hypot(swerveConfig.getTrackwidthMeters() / 2.0, swerveConfig.getWheelbaseMeters() / 2.0);
  public static final double MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      * 0.06; // 0.06 a magic number based on testing
  private double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 70.0;

  private static RobotConfig pathPlannerRobotConfig;

  // Standard deviations for poseEstimator updates
  // The wpilib matrix constructor requires sizes specified as Nat types.
  // Determined these settings from looking at other team's settings
  // we use a high variance for the camera yaw because we don't want it to
  // override
  // the odometry yaw, which comes from the very accurate IMU
  private Matrix<N3, N1> visionStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), new double[] { 0.7, 0.7, 100 });
  private Matrix<N3, N1> odometryStdDev = MatBuilder.fill(Nat.N3(), Nat.N1(), new double[] { .1, .1, 0.01 });

  private ArrayList<Double> recentVisionYaws = new ArrayList<Double>();
  private int recentVisionYawsMaxSize = 15;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private ChassisSpeeds previousChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private double speedReductionFactor = 1.0;

  private SwerveDriveMode swerveDriveMode = SwerveDriveMode.FIELD_CENTRIC_DRIVING;

  private TunerSwerveDrivetrain drivetrain = new TunerSwerveDrivetrain(TunerConstants.DrivetrainConstants, 0,
      odometryStdDev, visionStdDev,
      TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

  private SwerveRequest.FieldCentric fieldCentricDriveController = new SwerveRequest.FieldCentric()
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  private SwerveRequest.RobotCentric robotCentricDriveController = new SwerveRequest.RobotCentric()
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  private SwerveRequest.SwerveDriveBrake brakeDriveController = new SwerveRequest.SwerveDriveBrake();

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /**
   * Constructor for this DrivetrainSubsystem
   */
  public DrivetrainSubsystem(SubsystemCollection subsystems) {
    if(InstalledHardware.limelightInstalled){
      cameraSubsystem = subsystems.getCameraSubsystem();
    }

    timer.reset();
    timer.start();

    drivetrain.registerTelemetry(logger::telemeterize);

    publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // TODO change this to use the explicit config below (one debugged)
    // We don't want to rely on accessing PathPlanner GUI in a match. 
    // get Path Planner config from GUI settings.
    try{
      pathPlannerRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    /**
    pathPlannerRobotConfig = new RobotConfig(
      29.0, 
      4.2,
      new ModuleConfig(
        Constants.SWERVE_WHEEL_DIAMETER / 2,
        MAX_VELOCITY_METERS_PER_SECOND + 1.0, //PP doesn't like our low velocity?
        1.0, // default value
        // from CTRE Kraken x 60 https://ctre.download/files/datasheet/Motor%20Performance%20Analysis%20Report.pdf
        new DCMotor(12.0, 7.09,  374.38, 2.0, 6000 * 2 * Math.PI * 60, 1),
        1.0/SWERVE_DRIVE_REDUCTION,
        50,
        1));
    */
  }

  /**
   * Method avaliable so that callers can update the chassis speeds to induce
   * changes in robot movement
   * This method is field centric driving and applies acceleration limiter and
   * speed reduction factor
   * 
   * @param updatedChassisSpeeds - the updated chassis speeds (x, y and rotation)
   */
  public void driveFieldCentric(ChassisSpeeds updatedChassisSpeeds) {
    // if we are switching from robot centric to field centric, 
    // convert the previous chassis speeds to the correct frame of reference. 
    if (swerveDriveMode == SwerveDriveMode.ROBOT_CENTRIC_DRIVING){
      this.previousChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(previousChassisSpeeds, getGyroscopeRotation());
    }
    this.swerveDriveMode = SwerveDriveMode.FIELD_CENTRIC_DRIVING;
    this.chassisSpeeds = updatedChassisSpeeds;
    
  }

  /**
   * Method avaliable so that callers can update the chassis speeds to induce
   * changes in robot movement
   * This method is robot centric driving and does not apply any acceleration
   * limiter or speed reduction factor
   * It is intended to be used with a trajectory generator that imposes speed and
   * acceleration constraints.
   * 
   * @param updatedChassisSpeeds - the updated chassis speeds (x, y and rotation)
   */
  public void driveRobotCentric(ChassisSpeeds updatedChassisSpeeds) {
    // if we are switching from field centric to robot centric, 
    // convert the previous chassis speeds to the correct frame of reference. 
    if (swerveDriveMode == SwerveDriveMode.FIELD_CENTRIC_DRIVING){
      this.previousChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(previousChassisSpeeds, getGyroscopeRotation());
    }
    this.swerveDriveMode = SwerveDriveMode.ROBOT_CENTRIC_DRIVING;
    this.chassisSpeeds = updatedChassisSpeeds;
  }

  /**
   * returns chassis speeds (robot relative)
   * 
   * @return chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getState().Speeds;
  }

  /**
   * Method to get the current speed reduction factor
   * 
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
   * 
   * @param shouldUseVision true to use vision, false to not
   */
  public void setUseVision(boolean shouldUseVision) {
    this.useVision = shouldUseVision;
  }

  /**
   * Method to set the current speed reduction factor to a new value
   * 
   * @param value - the new speed reduction factor
   */
  public void setSpeedReductionFactor(double value) {
    this.speedReductionFactor = MotorUtils.truncateValue(value, 0.0, 1.0);
  }

  /**
   * Obtains the current gyroscope's rotation about the Z axis when looking down
   * at the robot where positive is measured in the
   * counter-clockwise direction.
   * 
   * @return A Rotation2d that describes the current orentation of the robot.
   */
  public Rotation2d getGyroscopeRotation() {
    return drivetrain.getState().Pose.getRotation();
  }

  /**
   * A method to get the current position of the robot
   * 
   * @return the current Pose2d position of the robot
   */
  public Pose2d getRobotPosition() {
    return drivetrain.getState().Pose;
  }

  /**
   * A method to obtain the swerve drive mode
   * 
   * @return the mode
   */
  public SwerveDriveMode getSwerveDriveMode() {
    return swerveDriveMode;
  }

  /**
   * Perodic for this subsystem - very important for it to run every scheduler
   * cycle - 50Hz
   */
  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if (DriverStation.isDisabled()) {
      if (!m_hasAppliedOperatorPerspective){
        DriverStation.getAlliance().ifPresent(allianceColor -> {
          drivetrain.setOperatorPerspectiveForward(
              allianceColor == Alliance.Red
                  ? kRedAlliancePerspectiveRotation
                  : kBlueAlliancePerspectiveRotation);
          m_hasAppliedOperatorPerspective = true;
        });
      }
      //TODO validate this at test field
      //When disabled countinually set the botpose to what the vision says
      
      this.seedRobotPositionFromVision();
    }

    // update robot position with vision
    if (InstalledHardware.limelightInstalled && DriverStation.isEnabled()) {
      this.addVisionMeasurement(cameraSubsystem.getVisionBotPose());
    }

    if (swerveDriveMode == SwerveDriveMode.IMMOVABLE_STANCE && chassisSpeedsAreZero()) {
      // only change to ImmovableStance if chassis is not moving.
      // otherwise, we could tip the robot moving to this stance when bot is at high
      // velocity
      drivetrain.setControl(brakeDriveController);
    } else if (swerveDriveMode == SwerveDriveMode.FIELD_CENTRIC_DRIVING) {
      // apply the speed reduction factor to the chassis speeds
      ChassisSpeeds reducedChassisSpeeds = new ChassisSpeeds(
          chassisSpeeds.vxMetersPerSecond * this.speedReductionFactor,
          chassisSpeeds.vyMetersPerSecond * this.speedReductionFactor,
          // different speed reduction factor for rotation
          chassisSpeeds.omegaRadiansPerSecond * Math.min(1.0, this.speedReductionFactor * 1.25));

      // apply acceleration control
      reducedChassisSpeeds = limitChassisSpeedsAccel(reducedChassisSpeeds);
      previousChassisSpeeds = reducedChassisSpeeds;

      // send the requested chassis speeds to the sweve drive
      drivetrain.setControl(fieldCentricDriveController
          .withVelocityX(reducedChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(reducedChassisSpeeds.vyMetersPerSecond)
          .withRotationalRate(reducedChassisSpeeds.omegaRadiansPerSecond));
    } else { // ROBOT_CENTRIC_DRIVING
      // do not apply acceleration limis or speed limits.
      // assumes this is used by a trajectory follower that already imposes those
      // constraints.
      drivetrain.setControl(robotCentricDriveController
          .withVelocityX(chassisSpeeds.vxMetersPerSecond)
          .withVelocityY(chassisSpeeds.vyMetersPerSecond)
          .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
    }
    displayDiagnostics();
  }

  /**
   * A method to set the current position of the robot
   * 
   * @param updatedPosition - the new position of the robot
   */
  public void setRobotPosition(Pose2d updatedPosition) {
    drivetrain.resetPose(updatedPosition);
  }

  /**
   * A method to set the swerve drive mode to immovable stance
   */
  public void setImmovableStance() {
    this.swerveDriveMode = SwerveDriveMode.IMMOVABLE_STANCE;
    // DO NOT set chassis speeds to 0.0 here.
    // we need to allow the robot to decel properly so that it doesn't tip.
  }

  /**
   * A method to unset the swerve drive from immovable stance
   */
  public void unsetImmovableStance() {
    // we can unset IMMOVABLE_STANCE by driving field centric at a speed of 0
    this.driveFieldCentric(new ChassisSpeeds(0, 0, 0));
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    drivetrain.seedFieldCentric();
  }

  /**
   * A method to zero the current position
   */
  public void zeroRobotPosition() {
    this.setRobotPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public void seedRobotPositionFromVision() {
    Pose2d visionBotPose = cameraSubsystem.getVisionBotPose().getRobotPosition();

    if (visionBotPose != null) {
      recentVisionYaws.add(visionBotPose.getRotation().getDegrees());
      while(recentVisionYaws.size() > recentVisionYawsMaxSize){
        recentVisionYaws.remove(0);
      }
      LimelightHelpers.SetRobotOrientation("", getMedianOfList(recentVisionYaws), 0, 0, 0, 0, 0);
      Pose2d visonBotPoseOrb = cameraSubsystem.getVisionBotPoseOrb().getRobotPosition();
      if (visonBotPoseOrb != null) {
        Pose2d combinedBotPose = new Pose2d(visonBotPoseOrb.getTranslation(), Rotation2d.fromDegrees(getMedianOfList(recentVisionYaws)));
        this.setRobotPosition(combinedBotPose);
      }
    }
  }

  public Double getMedianOfList(ArrayList<Double> list){
    ArrayList<Double> modifiedList = new ArrayList<Double>(list);
    Collections.sort(modifiedList);
    return modifiedList.get((int)(modifiedList.size()/2));
  }

  /**
   * A method that updates the robot position with a vision measurement
   * 
   * @param visionMeasurement the most recent vision measurement provided by
   *                          vision subsystem
   */
  private void addVisionMeasurement(VisionMeasurement visionMeasurement) {
    // for now ignore all vision measurements that are null or contained robot
    // position is null
    if (visionMeasurement != null && useVision) {
      Pose2d visionComputedMeasurement = visionMeasurement.getRobotPosition();
      if (visionComputedMeasurement != null) {
        // we want to reject vision measurements that are more than 1 meter away in case
        // vison gives a bad read
        furtherThanAMeter = visionComputedMeasurement.getTranslation().getDistance(getRobotPosition().getTranslation()) <= 2;
        if (visionComputedMeasurement.getTranslation().getDistance(getRobotPosition().getTranslation()) <= 2) {
          drivetrain.addVisionMeasurement(visionComputedMeasurement, Utils.fpgaToCurrentTime(visionMeasurement.getTimestamp()));
        }
      }
    }
  }

  private boolean chassisSpeedsAreZero() {
    return (chassisSpeeds.vxMetersPerSecond == 0.0)
        && (chassisSpeeds.vyMetersPerSecond == 0.0)
        && (chassisSpeeds.omegaRadiansPerSecond == 0.0);
  }

  /**
   * Clamps the chassis speeds between a min and max.
   * Separete min and max for translational (x,y) vs. rotational speeds
   * 
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
      double rotationMax) {
    // do not scale omega
    // double clampedOmega =
    // MotorUtils.doubleSidedClamp(chassisSpeeds.omegaRadiansPerSecond, rotationMin,
    // rotationMax);
    // if one or both of X or Y needs to be clamped, we need to scale both
    // proportionally
    // form a Translation2d of x, y so the math is easier
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double velocity = translation.getNorm();
    double clampedVelocity = MotorUtils.doubleSidedClamp(velocity, translationMin, translationMax);
    // scale the translation by the clamped velocity scale factor
    double scale = clampedVelocity / velocity;
    translation = translation.times(scale);
    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Limits chassis speeds based on max allowable acceleration
   * 
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
    double omegaVelocityLimited = limitAxisSpeed(speeds.omegaRadiansPerSecond,
        previousChassisSpeeds.omegaRadiansPerSecond,
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED,
        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    return new ChassisSpeeds(xVelocityLimited, yVelocityLimited, omegaVelocityLimited);
  }

  /**
   * Limits axis speed based on max allowable acceleration
   * 
   * @param commandedSpeed
   * @param previousSpeed
   * @param maxAccel
   * @return limited speed
   */
  private double limitAxisSpeed(double commandedSpeed, double previousSpeed, double maxAccel, double maxDecel) {
    double accel = (commandedSpeed - previousSpeed) / deltaTimeSeconds;
    double speedLimited = commandedSpeed;
    if (accel > maxAccel) { // positive accel is greater than max
      // new velocity is the old velocity + the maximum allowed change toward the new
      // direction
      speedLimited = previousSpeed + Math.copySign(maxAccel * deltaTimeSeconds, accel);
    } else if (accel < -maxDecel) { // accel is less than negative max
      speedLimited = previousSpeed + Math.copySign(maxDecel * deltaTimeSeconds, accel);
    }
    return speedLimited;
  }

  private void displayDiagnostics() {
    SmartDashboard.putNumber("RobotFieldHeadingDegrees", drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("RobotFieldXCoordinateMeters", drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("RobotFieldYCoordinateMeters", drivetrain.getState().Pose.getY());
    SmartDashboard.putBoolean("VisionWithinAMeter", furtherThanAMeter);
    SmartDashboard.putBoolean("UseVision", useVision);
    
    if (displayOdometryDiagnostics) {
      VisionMeasurement visionBotPose = cameraSubsystem.getVisionBotPose();
      if (visionBotPose.getRobotPosition() != null){
        SmartDashboard.putNumber("vision x", visionBotPose.getRobotPosition().getX());
        SmartDashboard.putNumber("vision y", visionBotPose.getRobotPosition().getY());
        SmartDashboard.putNumber("vision theta", visionBotPose.getRobotPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("vision timestamp", visionBotPose.getTimestamp());
        SmartDashboard.putNumber("current timestamp", Utils.fpgaToCurrentTime(visionBotPose.getTimestamp()));
        SmartDashboard.putNumber("robot timestamp", Timer.getFPGATimestamp());
      }
    }
  }
}