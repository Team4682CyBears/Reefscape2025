package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.signals.InvertedValue;

public class Spinner extends SubsystemBase {
    //configurtition for the motor begins here
    private TalonFX motorTalon;
    //velcity and voltage control
    private final VelocityVoltage motorTalonVelocityController = new VelocityVoltage(0);
    private final VoltageOut motorTalonVoltageController = new VoltageOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;
    private Slot0Configs motorRpmGains = new Slot0Configs().withKS(0.5);

    public Spinner(){
        motorTalon = new TalonFX(Constants.motorCanID);
        configureMotor();
        System.out.print("Motor is set");
        motorTalonVelocityController.withSlot(0);
    }

    public void spinMotor(double speed){
        //Sets speed to a specific state for testing
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(speed));
    }

    public void motorStop(){
        //stop voltage
        motorTalon.setControl(this.motorTalonVoltageController.withOutput(0));
    }
    
    public double getSpeed(){
        //return speed
        return motorTalon.getVelocity().getValueAsDouble();
    }

    private void configureMotor(){
        // Config motor
        motorTalonMotorConfiguration = new TalonFXConfiguration(); 
        motorTalonMotorConfiguration.Slot0 = motorRpmGains;
        // do not config feedbacksource, since the default is the internal one.
        motorTalonMotorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorTalonMotorConfiguration.Voltage.PeakReverseVoltage = -12;
        motorTalonMotorConfiguration.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;
        // maximum current settings
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        StatusCode response = motorTalon.getConfigurator().apply(this.motorTalonMotorConfiguration);
        if (!response.isOK()) {
          System.out.println(
              "TalonFX ID " + motorTalon.getDeviceID() + " failed config with error " + response.toString());
        }
        
    }
}
