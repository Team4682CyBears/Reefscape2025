// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: TalonMotor.java
// Intent: Configures and runs our TalonFX motor via duty cycle
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// ** 
// Documation for TalonFX motor:
// https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html#control-output-types
// ** 

//
// DutyCycle Description:
// 
// A DutyCycle control request outputs a proportion of the supply 
// voltage, which typically ranges from -1.0 to 1.0, inclusive. 
// This control output type is typically used in systems where it 
// is important to be capable of running at the maximum speed 
// possible, such as in a typical robot drivetrain.
//

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkBase.IdleMode;
//import com.revrobotics.spark.SparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.common.NoteTofSensor;
import frc.robot.Constants;
//import frc.robot.control.InstalledHardware;


public class NeoMotorSubsystem extends SubsystemBase {
    //configurtition for the motor begins here
    private SparkMax climberMotor = null;//

    public NeoMotorSubsystem() {

        climberMotor = new SparkMax(Constants.climberMotorCanId, MotorType.kBrushless);
    }

      /**
   * A method to set the intake speed
   * @param speed (between -1 and 1)
   */
  public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
  }


  /**
   * A method to stop the intake subsystem
   */
  public void setAllStop() {
    climberMotor.set(0.0);
  }
}
