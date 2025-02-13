// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ElevatorSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// namspace to contain class
package frc.robot.subsystems;

// import motor libraries from ctre
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import wpilib libraries
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import local classes
import frc.robot.Constants;
import frc.robot.common.CorrectableEncoderPlusDigitalIoPort;
import frc.robot.common.ElevatorDirection;
import frc.robot.common.ElevatorMovementMode;
import frc.robot.common.MotorUtils;
//import com.ctre.phoenix6.controls.StrictFollower;

// import wpilib libraries
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

//import javax.lang.model.util.ElementScanner14;

// elevator subs
public class ElevatorSubsystem extends SubsystemBase {

  // define class variables
  private double inchesPerRotation = 2.25; // four inches per rotation TODO update with actual value.
  private Distance startingPosition = Inches.of(0.0);
  private Distance heightTolerance = Inches.of(0.25);

  // define heights
  private final Distance sensorHeight = Inches.of(1.0);
  private final Distance maxHeight = Inches.of(74.0);
  private final Distance minHeight = Inches.of(0.0);

  // Create motor controller instances
  private TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorCANID);
  private TalonFX secondElevatorMotor = new TalonFX(Constants.secondMotorCanID);

  // controller for position-based movement
  private final MotionMagicExpoDutyCycle elevatorPositionalController = new MotionMagicExpoDutyCycle(
      distanceToAngle(startingPosition));

  // contoller for joystick-based movement
  private final DutyCycleOut elevatorJoystickController = new DutyCycleOut(0.0);

  private DigitalInput elevatorMageneticSensor = new DigitalInput(Constants.elevatorMageneticSensorID);
  private ElevatorMovementMode elevatorMovementMode = ElevatorMovementMode.STOPPED;

  // when moving to a set potision, use motionMagic to control speeds
  // when moving via a joystick, need to specify speeds.
  private final double elevatorRetractSpeed = -0.050;
  private final double elevatorExtendSpeed = .050;
  private ElevatorDirection elevatorDirection = ElevatorDirection.UP;
  private Distance targetHeight = startingPosition;

  // per documentation, need to set kV and kA (in duty cycle) 
      
  private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.00625).withKV(0.00885).withKP(0.2).withKA(0.001);
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs().withMotionMagicExpo_kA(0.1).withMotionMagicExpo_kV(0.11);
  private NeutralModeValue motorNeutralModeValue = NeutralModeValue.Brake;

  private CorrectableEncoderPlusDigitalIoPort elevatorCorrectableEncoder = null;

  public ElevatorSubsystem() {
    secondElevatorMotor.setControl(new StrictFollower(elevatorMotor.getDeviceID()));
    configureMotor();
    configurePositionalController();

    elevatorCorrectableEncoder = new CorrectableEncoderPlusDigitalIoPort(elevatorMotor,
        elevatorMageneticSensor,
        distanceToAngle(sensorHeight).in(Rotations),
        distanceToAngle(startingPosition).in(Rotations));
  }

  /*
   * A method to set elevator in a moveUp mode
   */
  public void moveUp() {
    elevatorDirection = ElevatorDirection.UP;
    elevatorMovementMode = ElevatorMovementMode.VELOCITY;
  }

  /*
   * A method to set elevator in a moveDown mode
   */
  public void moveDown() {
    elevatorDirection = ElevatorDirection.DOWN;
    elevatorMovementMode = ElevatorMovementMode.VELOCITY;
  }

  /*
   * A method to set the elevator in a moveToPosition mode
   */
  public void moveToPosition(Distance targetPosition) {
    this.targetHeight = Inches
        .of(MotorUtils.clamp(targetPosition.in(Inches), minHeight.in(Inches), maxHeight.in(Inches)));
    elevatorMovementMode = ElevatorMovementMode.POSITION;
  }

  /*
   * A method to set elevator in a stop mode
   */
  public void stopElevator(){
    elevatorMovementMode = ElevatorMovementMode.STOPPED;
  }

  /*
   * returns the current height as a distance
   */
  public Distance getCurrentHeight() {
    return angleToDistance(elevatorCorrectableEncoder.getCurrentEncoderPosition());
  }

  /*
   * returns true when the elevator is within a certain tolerance of the target height
   */
  public boolean isAtTargetHeight() {
    Distance currentHeight = getCurrentHeight();
    return currentHeight.isNear(targetHeight, heightTolerance);
  }

  /*
   * periodic for the elevator
   */
  public void periodic() {
    elevatorCorrectableEncoder.updateEncoderPosition();
    Distance currentHeight = getCurrentHeight();
    if (elevatorMovementMode == ElevatorMovementMode.VELOCITY &&
        elevatorDirection == ElevatorDirection.UP &&
        currentHeight.lt(maxHeight)) {
          elevatorJoystickController.withOutput(elevatorExtendSpeed);
          elevatorMotor.setControl(elevatorJoystickController);
        
        }
        else if(elevatorMovementMode == ElevatorMovementMode.VELOCITY &&
    elevatorDirection == ElevatorDirection.DOWN &&
    currentHeight.gt(minHeight)) {
        elevatorJoystickController.withOutput(elevatorRetractSpeed);
        elevatorMotor.setControl(elevatorJoystickController); 
    }
    else if(elevatorMovementMode == ElevatorMovementMode.POSITION && !isAtTargetHeight()){
      elevatorPositionalController.withPosition(distanceToAngle(targetHeight));  
      elevatorMotor.setControl(elevatorPositionalController);
    }
    else {
      elevatorMovementMode = ElevatorMovementMode.STOPPED;
      elevatorMotor.stopMotor();
    }
    SmartDashboard.putNumber("elevator position", getCurrentHeight().in(Inches));
  }

  /*
   * converts an rotations to inches
   */
  public Distance angleToDistance(Angle anglePosition) {
    return Inches.of(anglePosition.in(Rotations) * inchesPerRotation);

  } 

  /*
   * configures motor
   */
     private void configureMotor(){
        // Config motor
        TalonFXConfiguration motorTalonMotorConfiguration = new TalonFXConfiguration(); 
        motorTalonMotorConfiguration.withMotionMagic(motionMagicConfigs);
        motorTalonMotorConfiguration.MotorOutput.NeutralMode = this.motorNeutralModeValue;
        motorTalonMotorConfiguration.Slot0 = slot0Configs;
        motorTalonMotorConfiguration.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.02);
        // do not config feedbacksource, since the default is the internal one.
        motorTalonMotorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorTalonMotorConfiguration.Voltage.PeakReverseVoltage = -12;
        motorTalonMotorConfiguration.Voltage.SupplyVoltageTimeConstant = 0.02;

        // maximum current settings
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // min and max position software limit (set into the motor encoder, itself)
        motorTalonMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorTalonMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = distanceToAngle(maxHeight).in(Rotations);
        motorTalonMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorTalonMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = distanceToAngle(minHeight).in(Rotations);

        StatusCode response = elevatorMotor.getConfigurator().apply(motorTalonMotorConfiguration);
        if (!response.isOK()) {
          System.out.println(
              "TalonFX ID " + elevatorMotor.getDeviceID() + " failed config with error " + response.toString());
        }

        // change invert for angleRightMotor
        motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      // apply configs
      response = secondElevatorMotor.getConfigurator().apply(motorTalonMotorConfiguration);
      if (!response.isOK()) {
        DataLogManager.log(
            "TalonFX ID " + secondElevatorMotor.getDeviceID() + " failed config with error " + response.toString());
      }
    
    }

    /*
   * configures positional controller
   */
  private void configurePositionalController(){
    elevatorPositionalController.UpdateFreqHz = 0;
    elevatorPositionalController.OverrideBrakeDurNeutral = true;
    elevatorPositionalController.UseTimesync = false;
    elevatorPositionalController.LimitForwardMotion = false;
    elevatorPositionalController.LimitReverseMotion = false;
    elevatorPositionalController.Slot = 0;
  }

  /*
   * converts inches to rotations
   */
  public Angle distanceToAngle(Distance distancePosition) {
    return Rotations.of(distancePosition.in(Inches) / inchesPerRotation);

  }

}