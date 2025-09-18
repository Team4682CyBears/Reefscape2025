// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ElevatorSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.common.CorrectableEncoderPlusDigitalIoPort;
import frc.robot.common.ElevatorDirection;
import frc.robot.common.ElevatorMovementMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.MotorUtils;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

//import javax.lang.model.util.ElementScanner14;

public class ElevatorSubsystem extends SubsystemBase {

  // define class variables
  private double inchesPerRotation = 0.8; // a combination of the wheel circumference and the gearing.
  // We make the assumption that the elevator starts below the mag sensor.
  // It doesn't matter exactly where below the mag sensor.
  // Once it moves up past the mag sensor, then it will have its correct position.
  // All set points are above the starting position, so any call to move to
  // position
  // will cause it to move past the mag sensor.
  private Distance startingPosition = Inches.of(0.0);
  private Distance heightTolerance = Inches.of(0.25);
  private final Distance sensorHeight = Inches.of(1.0);
  private final Distance maxHeight = Inches.of(74.0);
  private final Distance minHeight = Inches.of(0.0);

  private TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorLeaderCanID);
  private TalonFX secondElevatorMotor = new TalonFX(Constants.elevatorMotorFollowerCanID);
  // controller for position-based movement
  private final MotionMagicExpoDutyCycle elevatorPositionalController = new MotionMagicExpoDutyCycle(
      distanceToAngle(startingPosition));
  // contoller for joystick-based movement
  private final DutyCycleOut elevatorJoystickController = new DutyCycleOut(0.0);

  private DigitalInput elevatorMageneticSensor = new DigitalInput(Constants.elevatorMageneticSensorID);
  private ElevatorMovementMode elevatorMovementMode = ElevatorMovementMode.STOPPED;
  private ElevatorMovementMode previousElevatorMovementMode = ElevatorMovementMode.STOPPED;

  // when moving to a set potision, use motionMagic to control speeds
  // when moving via a joystick, need to specify speeds.
  // initial retract speed is very low until the mag sensor has been tripped.
  private final double elevatorSlowRetractSpeed = -0.050;
  private final double elevatorRetractSpeed = -0.1;
  private final double elevatorExtendSpeed = .1;
  private ElevatorDirection elevatorDirection = ElevatorDirection.UP;
  private Distance targetHeight = startingPosition;

  // per documentation, need to set kV and kA (in duty cycle)

  private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.00625).withKV(0.00885).withKP(0.18).withKA(0.001);
  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs().withMotionMagicExpo_kA(0.1)
      .withMotionMagicExpo_kV(0.11);
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
   * returns the current height as a distance
   */
  public Distance getCurrentHeight() {
    return angleToDistance(elevatorCorrectableEncoder.getCurrentEncoderPosition());
  }

  /**
   * returns true when the elevator is within a certain tolerance of the stow
   * position
   * 
   * @return
   */
  public boolean isAtStow() {
    return getCurrentHeight().isNear(Constants.stowHeight, heightTolerance);
  }

  /*
   * returns true when the elevator is within a certain tolerance of the target
   * height
   */
  public boolean isAtTargetHeight() {
    Distance currentHeight = getCurrentHeight();
    return currentHeight.isNear(targetHeight, heightTolerance);
  }

  /*
   * A method to set elevator in a moveDown mode
   * if the mag sensor has not yet been tripped, elevator will not move down.
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
        .of(MotorUtils.clamp(
            targetPosition.in(Inches),
            minHeight.in(Inches),
            maxHeight.in(Inches)));
    // TODO fix this logic. 
    // Why are we doing it this way?
    // We believe this is causing a small jump in the position when it stops. 
    if (getCurrentHeight().gt(targetPosition)) {
      targetPosition = targetPosition.minus(Inches.of(0.5));
    } else if (getCurrentHeight().lt(targetPosition)) {
      targetPosition = targetPosition.plus(Inches.of(0.5));
    }
    elevatorMovementMode = ElevatorMovementMode.POSITION;
  }

  /*
   * A method to set elevator in a moveUp mode
   */
  public void moveUp() {
    elevatorDirection = ElevatorDirection.UP;
    elevatorMovementMode = ElevatorMovementMode.VELOCITY;
  }

  /*
   * periodic for the elevator
   */
  public void periodic() {

    // Correct encoder position
    elevatorCorrectableEncoder.updateEncoderPosition();

    // get current height
    Distance currentHeight = getCurrentHeight();

    //
    SmartDashboard.putNumber("ElevatorHeight", getCurrentHeight().in(Inches));
    SmartDashboard.putNumber("ElevatorTargetPosition", targetHeight.in(Inches));

    // check velcity and up mode (and below max height)
    if (elevatorMovementMode == ElevatorMovementMode.VELOCITY &&
        elevatorDirection == ElevatorDirection.UP &&
        currentHeight.lt(maxHeight)) {

      // run motor
      elevatorJoystickController.withOutput(elevatorExtendSpeed);
      elevatorMotor.setControl(elevatorJoystickController);

      // check if velocity and down mode (and above min height)
    } else if (elevatorMovementMode == ElevatorMovementMode.VELOCITY &&
        elevatorDirection == ElevatorDirection.DOWN &&
        currentHeight.gt(minHeight)) {

      // if the mag sensor has not ever been tripped, go down slowly!
      elevatorJoystickController.withOutput(getElevatorRetractSpeed());
      elevatorMotor.setControl(elevatorJoystickController);

      // check if elevator is in position mode, and is not at target height
    } else if (elevatorMovementMode == ElevatorMovementMode.POSITION && !isAtTargetHeight()) {

      // run motor
      elevatorPositionalController.withPosition(distanceToAngle(targetHeight));
      elevatorMotor.setControl(elevatorPositionalController);

      // if the elevator is in position mode and is at target position or if the
      // movement mode is stopped
    } else {
      // if (elevatorMovementMode != ElevatorMovementMode.STOPPED) {
      if (previousElevatorMovementMode != ElevatorMovementMode.STOPPED) {
        // if we have just changed into STOPPED state, set the hold position
        targetHeight = getCurrentHeight();
      }

      elevatorMovementMode = ElevatorMovementMode.STOPPED;

      // Updates the motor controller to go to targetHeight
      elevatorPositionalController.withPosition(distanceToAngle(targetHeight));
      elevatorMotor.setControl(elevatorPositionalController);
    }
    SmartDashboard.putNumber("elevator position", getCurrentHeight().in(Inches));
    previousElevatorMovementMode = elevatorMovementMode;
  }

  /*
   * converts an rotations to inches
   */
  public Distance angleToDistance(Angle anglePosition) {
    return Inches.of(anglePosition.in(Rotations) * inchesPerRotation);

  }

  /*
   * A method to set elevator in a stop mode
   */
  public void stopElevator() {
    elevatorMovementMode = ElevatorMovementMode.STOPPED;
  }

  /*
   * configures motor
   */
  private void configureMotor() {
    // Config motor
    TalonFXConfiguration motorTalonMotorConfiguration = new TalonFXConfiguration();
    motorTalonMotorConfiguration.withMotionMagic(motionMagicConfigs);
    motorTalonMotorConfiguration.MotorOutput.NeutralMode = this.motorNeutralModeValue;
    motorTalonMotorConfiguration.Slot0 = slot0Configs;
    motorTalonMotorConfiguration.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.02);
    // do not config feedbacksource, since the default is the internal one.
    motorTalonMotorConfiguration.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
    ;
    motorTalonMotorConfiguration.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
    motorTalonMotorConfiguration.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

    // maximum current settings
    motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
    motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
    motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // motor direction
    motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // min and max position software limit (set into the motor encoder, itself)
    motorTalonMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorTalonMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = distanceToAngle(maxHeight)
        .in(Rotations);
    motorTalonMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorTalonMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = distanceToAngle(minHeight)
        .in(Rotations);

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
  private void configurePositionalController() {
    elevatorPositionalController.UpdateFreqHz = 0;
    // elevatorPositionalController.OverrideBrakeDurNeutral = true;
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

  // if the mag sensor has not ever been tripped, go down slowly!
  private double getElevatorRetractSpeed() {
    if (elevatorCorrectableEncoder.getMotorEncoderEverReset()) {
      return elevatorSlowRetractSpeed;
    } else {
      return elevatorRetractSpeed;
    }
  }

}