// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterAngleSubsystem.java
// Intent: Forms the prelminary code for shooter.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// package containing class
package frc.robot.subsystems;

// ctre library imports for controlling motor
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// wpilib import for math & logging
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// wpilib import for base class for subsystems
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// local class imports
import frc.robot.Constants;
import frc.robot.HardwareConstants;
import frc.robot.InstalledHardware;
import frc.robot.common.MotorUtils;
import frc.robot.common.ShooterPosition;

/**
 * Forms a class for the shooter subsystem
 * Consists of two outfeed motors, 
 * 1 angle motor, and 1 angle encoder
 */
public class ShooterAngleSubsystem extends SubsystemBase {

  // Initiate fields and class instances

  // Shooter gearing 
  private static final double angleEncoderGearRatio = 1.0; // angle encoder is mounted directly onto shaft
  // unused gear ratio but may need later
  //private static final double angleMotorGearRatio = 4.0; // 4:1 (12 -> 48) 

  // Tolerance
  // rotations per second (max 512)
  private static final double shooterAngleLowVelocityTol = 10;   

  // CanIds to "talk" to hardware
  private TalonFX angleMotor = new TalonFX(Constants.shooterMotorCanId);
  private CANcoder angleEncoder = new CANcoder(Constants.shooterEncoderCanId);

  // control class
  private final MotionMagicVoltage angleVoltageController = new MotionMagicVoltage(0);

  // don't start moving until angle is set. 
  private boolean shooterIsAtDesiredAngle = true; 
  private double desiredAngleDegrees; 

  // Motor controller gains
  private Slot0Configs angleMotorGainsForAbsoluteEncoder = new Slot0Configs()
    .withKP(Constants.kp)
    .withKI(Constants.ki)
    .withKD(Constants.kd)
    .withKV(Constants.kv);

  /**
   * Constructor for shooter subsystem
   */
  public ShooterAngleSubsystem() {
    configureAngleEncoder();
    configureAngleMotors();  
    /* Make control requests synchronous */
    angleVoltageController.UpdateFreqHz = 0;
  }

  /**
   * A method to get the shooter angle
   * @return angle in degrees
   */
  public double getAngleDegrees(){
    return rotationsToDegrees(angleMotor.getPosition().getValue()) + getOffset();
  }

  /**
   * A method to test whether the angle is within tolerance of the target angle
   * @param targetAngleDegrees
   * @return true if the angle is within tolerance
   */
  public boolean isAngleWithinTolerance(double targetAngleDegrees){
    // check both the position and velocity. To allow PID to not stop before settling. 
    boolean positionTargetReached = Math.abs(getAngleDegrees() - targetAngleDegrees) < Constants.shooterAngleToleranceDegrees;

    boolean velocityIsSmall = Math.abs(angleMotor.getVelocity().getValue().in(Units.DegreesPerSecond)) < shooterAngleLowVelocityTol;

    return positionTargetReached && velocityIsSmall;
  }

  /**
   * this method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    if (!shooterIsAtDesiredAngle) {
        // use motionMagic voltage control
        angleMotor.setControl(angleVoltageController.withPosition(degreesToRotations(desiredAngleDegrees - getOffset())));
        // angleRightMotor acts as a follower
        // keep moving until it reaches target angle
        shooterIsAtDesiredAngle = isAngleWithinTolerance(desiredAngleDegrees);
    }
    SmartDashboard.putNumber("Shooter Absolute Angle Degrees", rotationsToDegrees(angleEncoder.getPosition().getValue()));
    SmartDashboard.putNumber("Shooter Motor Encoder Degrees", getAngleDegrees());
    //SmartDashboard.putNumber("Shooter Angle Motor Rotations ", angleMotor.getPosition().getValue());
  }

  /**
   * A method to translate shooter positions into degrees
   * @param position
   * @return degrees
   */
  public double positionToDegrees(ShooterPosition position){
    double angle = Constants.shooterAngle;
    switch (position) {
      case algae:
        angle = Constants.algaeAngle;
        break;
      case shooter:
        angle = Constants.shooterAngle;
        break;
      default:
        angle = Constants.shooterAngle;
        break;
    }
    return angle;
  }

  /**
   * A method to set the shooter angle
   * @param degrees
   */
  public void setAngleDegrees(double degrees){
    // DataLogManager.log("Setting Shooter Angle to " + degrees + " degrees.");
    double clampedDegrees = MotorUtils.clamp(degrees, degrees + 3, degrees - 3);
    if (clampedDegrees != degrees){
      DataLogManager.log("Warning: Shooter Angle requested degrees of " + degrees + 
      "exceeded bounds of [" + (degrees + 3) + " .. " + (degrees - 3) +
      "]. Clamped to " + clampedDegrees + ".");
    }
    desiredAngleDegrees = clampedDegrees;
    shooterIsAtDesiredAngle = isAngleWithinTolerance(desiredAngleDegrees);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {
  }

  private void configureAngleEncoder() {
    // Config CanCoder
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();

    // Create MagnetSensorConfigs and set the AbsoluteSensorRange to Unsigned 0 to 1
    encoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    //MagnetSensor.AbsoluteSensorDiscontinuityPoint.Unsigned_0To1;
    encoderConfigs.MagnetSensor.MagnetOffset = degreesToRotations(Constants.shooterAbsoluteAngleOffsetDegrees);
    encoderConfigs.MagnetSensor.SensorDirection = Constants.shooterAngleSensorDirection;
    // apply configs
    StatusCode response = angleEncoder.getConfigurator().apply(encoderConfigs);
    if (!response.isOK()) {
      DataLogManager.log(
        "CANcoder ID " + angleEncoder.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  private void configureAngleMotors() {
    // Config angle motor
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.MotorOutput.Inverted = Constants.angleLeftTalonShooterMotorDefaultDirection;
    angleConfigs.CurrentLimits.StatorCurrentLimit = HardwareConstants.shooterAngleStatorCurrentMaximumAmps;
    angleConfigs.CurrentLimits.StatorCurrentLimitEnable = true; 
    angleConfigs.CurrentLimits.SupplyCurrentLimit = HardwareConstants.shooterAngleSupplyCurrentMaximumAmps;
    angleConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleConfigs.Voltage.SupplyVoltageTimeConstant = HardwareConstants.shooterAngleSupplyVoltageTimeConstant;
    // FeedbackConfigs and offsets
    if (InstalledHardware.shooterAngleCanCoderInstalled) {
      DataLogManager.log("Configuring Shooter Angle Motor with CanCoder Feedback.");
      angleConfigs.Slot0 = angleMotorGainsForAbsoluteEncoder;
      angleConfigs.Feedback.SensorToMechanismRatio = angleEncoderGearRatio;
      angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      angleConfigs.Feedback.FeedbackRemoteSensorID = Constants.shooterEncoderCanId;
      // offset is set in CanCoder config above
    } 
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = 800.0;
    angleConfigs.MotionMagic.MotionMagicAcceleration = 160;
    angleConfigs.MotionMagic.MotionMagicJerk = 800; 
    // apply configs
    StatusCode response = angleMotor.getConfigurator().apply(angleConfigs);
    if (!response.isOK()) {
      DataLogManager.log(
          "TalonFX ID " + angleMotor.getDeviceID() + " failed config with error " + response.toString());
    }
    // change invert for angleRightMotor
    angleConfigs.MotorOutput.Inverted = Constants.angleRightTalonShooterMotorDefaultDirection;
    // apply configs
    }

  private double getOffset(){
    return 0;
  }

  private double degreesToRotations(double degrees)
  {
    return degrees/360;
  }

  private double rotationsToDegrees(Angle rotations)
  {
    return rotations.in(Units.Degrees); // Convert the angle to degrees
  }

}