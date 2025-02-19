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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonMotorSubsystem extends SubsystemBase {
    //configurtition for the motor begins here
    private TalonFX talonMotor;

    //velcity and voltage control
    private DutyCycleOut motorTalonDutyOut = new DutyCycleOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;

    // values found through manual system characterization using Phoenix Tuner X.
    private double speedRpm = 0.0;
    private NeutralModeValue motorTargetNeutralModeValue = NeutralModeValue.Brake;
    private static final double kMinDeadband = 0.001;

    public TalonMotorSubsystem(){

        // initialize devices on the rio can bus
        talonMotor = new TalonFX(Constants.motorCanID);
        configureMotor();
        System.out.print("Motor is set");
        motorTalonDutyOut.withUpdateFreqHz(0); // send commands instantly
    }

    public void spinMotor(double speedRpm){
        //Sets speed to a specific state for testing
        motorTalonDutyOut.Output = 0.4;
        talonMotor.setControl(this.motorTalonDutyOut);
        System.out.println(" variable speed is " + speedRpm);
        //System.out.println("NewMotor should be " + SmartDashboard.getNumber("Set Motor Speed", -1000)); //set a ridiculous default value so we will really know if it didn't load
    }

    public void motorStop(){
        //stop voltage
        speedRpm = 0;
        motorTalonDutyOut.Output = 0;
        talonMotor.setControl(this.motorTalonDutyOut);
        System.out.println("Motor stopped");
    }
    
    public double getSpeed(){
        //return speed
        return talonMotor.getVelocity().getValueAsDouble();
    }

    private void configureMotor(){
        // Config motor
        motorTalonMotorConfiguration = new TalonFXConfiguration(); 
        motorTalonMotorConfiguration.MotorOutput.NeutralMode = this.motorTargetNeutralModeValue;
        motorTalonMotorConfiguration.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
        motorTalonMotorConfiguration.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.02);
        // do not config feedbacksource, since the default is the internal one.
        motorTalonMotorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorTalonMotorConfiguration.Voltage.PeakReverseVoltage = -12;

        // maximum current settings
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        StatusCode response = talonMotor.getConfigurator().apply(this.motorTalonMotorConfiguration);
        if (!response.isOK()) {
          System.out.println(
              "TalonFX ID " + talonMotor.getDeviceID() + " failed config with error " + response.toString());
        }
    
    }
}
