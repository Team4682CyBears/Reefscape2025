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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.CorrectableEncoderPlusDigitalIoPort;
import frc.robot.common.MotorUtils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.util.*;

//import javax.lang.model.util.ElementScanner14;

public class ElevatorSubsystem extends SubsystemBase
{
  private double positionReset = 0;
  private TalonFX elevatorMotor = null;
  private final MotionMagicExpoDutyCycle elevatorDutyCycle = new MotionMagicExpoDutyCycle(positionReset);
  private final static double magHeight = Units.metersToInches(0.0);
  private final static double maxHeight = Units.metersToInches(0.0);
  private final static double minHeight = Units.metersToInches(0.0);
  
    private DigitalInput elevatorMageneticSensor = null;
    private static AngleUnit unit = Rotation;
    
        //took from arm, change when we get actual numbers
        private static final double elevatorSensorResetRetractSpeed = -0.7;
        private static final double elevatorSensorResetExtendSpeed = 1.0;
        private static double speed = 0.0;
        
          private Slot0Configs elevatorMotorGainsForInternalEncoder = new Slot0Configs().withKP(350).withKI(0).withKD(50.0).withKV(0);
          private Slot0Configs elevatorMotorGainsForAbsoluteEncoder = new Slot0Configs().withKP(150).withKI(0.125).withKD(0.05).withKV(0);
      
          private CorrectableEncoderPlusDigitalIoPort elevatorCorrectableEncoder = null;
        
          public ElevatorSubsystem(){
            elevatorMotor = new TalonFX(Constants.elevatorMotorCANID);
            elevatorMageneticSensor = new DigitalInput(Constants.elevatorMageneticSensorID);
    
            elevatorDutyCycle.UpdateFreqHz = 0;
            elevatorDutyCycle.OverrideBrakeDurNeutral = true;
            elevatorDutyCycle.UseTimesync = false;
            elevatorDutyCycle.LimitForwardMotion = false;
            elevatorDutyCycle.LimitReverseMotion = false;
      
            elevatorCorrectableEncoder = new CorrectableEncoderPlusDigitalIoPort(elevatorMotor, 
            elevatorMageneticSensor, 
            ElevatorSubsystem.magHeight,
            ElevatorSubsystem.minHeight,
            ElevatorSubsystem.maxHeight);
          
        }
      
        private void ElevatorSensorReset(){
          if(this.elevatorCorrectableEncoder.getMotorEncoderEverReset() == false){
            speed =
                  (this.elevatorMageneticSensor.get() == false) ?
                  ElevatorSubsystem.elevatorSensorResetExtendSpeed :
                  ElevatorSubsystem.elevatorSensorResetRetractSpeed;
          }
    
      }
    
      private void elevatorMoveUp(){
        elevatorMotor.set(elevatorSensorResetExtendSpeed);
        if(elevatorMotor.convertRotationsToInches(elevatorCorrectableEncoder.getCurrentEncoderPosition()) == ElevatorSubsystem.maxHeight){
          speed = 0.0;
          elevatorMotor.set(speed);
        }
      }

    
      //TODO get mechanism + gear ratio
      private double convertRotationsToInches(StatusSignal targetPosition){
        
        elevatorMotor.getPosition().getValue().in(Rotations);
        //return unit.convertFrom(elevatorSensorResetExtendSpeed, targetPosition) / Constants.TalonFXEncoderTicksPerRevolution;
  }


}