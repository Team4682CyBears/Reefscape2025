// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: WristSubsystem.java
// Intent: Forms the prelminary code for wrist.
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

// wpilib import for base class for subsystems
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// local class imports
import frc.robot.control.Constants;
import frc.robot.control.HardwareConstants;
import frc.robot.control.InstalledHardware;
import frc.robot.common.MotorUtils;
import frc.robot.common.WristPosition;

/**
 * Forms a class for the wrist subsystem
 * Consists of two outfeed motors, 
 * 1 angle motor, and 1 angle encoder
 */
public class WristSubsystem extends SubsystemBase {

  // Initiate fields and class instances

  // Wrist gearing 
  private static final double angleEncoderGearRatio = 1.0; // angle encoder is mounted directly onto shaft
  private static final double angleMotorGearRatio = 4.0; // 4:1 (12 -> 48) 

  // Tolerance
  // rotations per second (max 512)
  private static final double wristLowVelocityTol = 10;   

  // CanIds to "talk" to hardware
  private TalonFX angleMotor = new TalonFX(Constants.wristMotorCanId);
  private CANcoder angleEncoder = new CANcoder(Constants.wristEncoderCanId);

  // control class
  private final MotionMagicVoltage angleVoltageController = new MotionMagicVoltage(0);

  // don't start moving until angle is set. 
  private boolean wristIsAtDesiredAngle = true; 
  private double desiredAngleDegrees; 

  // Motor controller gains
  // copying gains from CoralTof miniproject.
  private Slot0Configs angleMotorGainsForAbsoluteEncoder = new Slot0Configs()
    .withKS(0.09)
    .withKV(0.45)
    .withKP(0.35)
    .withKD(0.0) // DO NOT SET KD!!
    .withKI(0.5);

  /**
   * Constructor for wrist subsystem
   */
  public WristSubsystem() {

    configureAngleEncoder();

    configureAngleMotors();  

    /* Make control requests synchronous */
    angleVoltageController.UpdateFreqHz = 0;
  }

  /**
   * A method to get the wrist angle
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
    boolean positionTargetReached = Math.abs(getAngleDegrees() - targetAngleDegrees) < Constants.wristToleranceDegrees;

    boolean velocityIsSmall = Math.abs(angleMotor.getVelocity().getValue().in(Units.DegreesPerSecond)) < wristLowVelocityTol;

    return positionTargetReached && velocityIsSmall;
  }

  /**
   * this method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    if (!wristIsAtDesiredAngle) {
        // use motionMagic voltage control
        angleMotor.setControl(angleVoltageController.withPosition(degreesToRotations(desiredAngleDegrees - getOffset())));
        // keep moving until it reaches target angle
        wristIsAtDesiredAngle = isAngleWithinTolerance(desiredAngleDegrees);
    }
  }

  /**
   * A method to translate wrist positions into degrees
   * @param position
   * @return degrees
   */
  public double positionToDegrees(WristPosition position){
    double angle = Constants.coralAngle;
    switch (position) {
      case ALGAE:
        angle = Constants.algaeAngle;
        break;
      case CORAL:
        angle = Constants.coralAngle;
        break;
    }
    return angle;
  }

  /**
   * A method to set the wrist angle
   * @param degrees
   */
  public void setAngleDegrees(double degrees){
    DataLogManager.log("Setting Wrist Angle to " + degrees + " degrees.");

    // check if set angle is in between max and min angle range
    double clampedDegrees = MotorUtils.clamp(degrees, Constants.wristMinDegrees, Constants.wristMaxDegrees);
    
    // if out of range, return warning that we outta range
    if (clampedDegrees != degrees){
      DataLogManager.log("Warning: Wrist Angle requested degrees of " + degrees + 
      "exceeded bounds of [" + Constants.wristMinDegrees + " .. " + Constants.wristMaxDegrees +
      "]. Clamped to " + clampedDegrees + ".");
    }

    // set desired angle based on clamped min/max
    desiredAngleDegrees = clampedDegrees;

    // cehck if angle within tolerance
    wristIsAtDesiredAngle = isAngleWithinTolerance(desiredAngleDegrees);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * This method sets config parameters for encoder
   */
  private void configureAngleEncoder() {
    // Config CanCoder
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();

    // Create MagnetSensorConfigs and set the AbsoluteSensorRange to Unsigned 0 to 1
    encoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderConfigs.MagnetSensor.MagnetOffset = degreesToRotations(Constants.wristAbsoluteAngleOffsetDegrees);
    encoderConfigs.MagnetSensor.SensorDirection = Constants.wristSensorDirection;
    // apply configs
    StatusCode response = angleEncoder.getConfigurator().apply(encoderConfigs);
    if (!response.isOK()) {
      DataLogManager.log(
        "CANcoder ID " + angleEncoder.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  /**
   * This method sets configuration parameters for motors
   */
  private void configureAngleMotors() {
    // Config angle motor
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.MotorOutput.Inverted = Constants.angleTalonWristMotorDefaultDirection;
    angleConfigs.CurrentLimits.StatorCurrentLimit = HardwareConstants.wristStatorCurrentMaximumAmps;
    angleConfigs.CurrentLimits.StatorCurrentLimitEnable = true; 
    angleConfigs.CurrentLimits.SupplyCurrentLimit = HardwareConstants.wristSupplyCurrentMaximumAmps;
    angleConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleConfigs.Voltage.SupplyVoltageTimeConstant = HardwareConstants.wristSupplyVoltageTimeConstant;

    // FeedbackConfigs and offsets
    if (InstalledHardware.wristCanCoderInstalled) {
      DataLogManager.log("Configuring Wrist Angle Motor with CanCoder Feedback.");
      angleConfigs.Slot0 = angleMotorGainsForAbsoluteEncoder;
      angleConfigs.Feedback.SensorToMechanismRatio = angleEncoderGearRatio;
      angleConfigs.Feedback.RotorToSensorRatio =   angleMotorGearRatio;
      angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      angleConfigs.Feedback.FeedbackRemoteSensorID = Constants.wristEncoderCanId;
      // offset is set in CanCoder config above
    } 

    // set motion magic speed configs
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.cruiseVelocity;
    angleConfigs.MotionMagic.MotionMagicAcceleration = Constants.acceleration;
    angleConfigs.MotionMagic.MotionMagicJerk = Constants.jerk; 

    // apply configs
    StatusCode response = angleMotor.getConfigurator().apply(angleConfigs);
    if (!response.isOK()) {
      DataLogManager.log(
          "TalonFX ID " + angleMotor.getDeviceID() + " failed config with error " + response.toString());
    }
    }

  /**
   * This method sets offset of the motor
   */
  private double getOffset(){
    //  the offset is 0 when we are using the fused motor/can coder.
    return 0;
  }

  /**
   * This method converts degrees to rotations
   */
  private double degreesToRotations(double degrees)
  {
    return degrees/360;
  }

  /**
   * This method converts rotations to degrees
   */
  private double rotationsToDegrees(Angle rotations)
  {
    return rotations.in(Units.Degrees); // Convert the angle to degrees
  }

}