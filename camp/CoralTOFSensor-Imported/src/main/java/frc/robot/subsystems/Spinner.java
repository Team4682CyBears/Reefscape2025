package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Spinner extends SubsystemBase {
    //configurtition for the motor begins here
    private TalonFX motorTalon;
    //velcity and voltage control
    private VelocityVoltage motorTalonVelocityController = new VelocityVoltage(0);
    private VoltageOut motorTalonVoltageController = new VoltageOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;
    //private Slot0Configs motorRpmGains = new Slot0Configs().withKS(0.2).withKP(0.11).withKI(0.001).withKD(0.001).withKV(0.12);
    private Slot0Configs motorRpmGains = new Slot0Configs().withKS(0.15).withKV(5.0).withKP(0.2); // this value of P seems too low. with P=0 it actualy works better
    private double speedRpm = 0.0;
    private NeutralModeValue motorTargetNeutralModeValue = NeutralModeValue.Coast;
    private static final double kMinDeadband = 0.001;

    public Spinner(){
        motorTalon = new TalonFX(Constants.motorCanID);
        configureMotor();
        System.out.print("Motor is set");
        motorTalonVelocityController.withSlot(0);
        motorTalonVelocityController.withUpdateFreqHz(0); // send commands instantly
    }

    public void spinMotor(double speed){
        //Sets speed to a specific state for testing
        speedRpm = speed;
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(speedRpm/60));
        //System.out.println(" variable speed is " + speedRpm);
        //System.out.println("NewMotor should be " + SmartDashboard.getNumber("Set Motor Speed", -1000)); //set a ridiculous default value so we will really know if it didn't load
    }

    public void motorStop(){
        //stop voltage
        speedRpm = 0;
        motorTalon.setControl(this.motorTalonVoltageController.withOutput(0));
        System.out.println("Motor stopped");
    }
    
    public double getSpeed(){
        //return speed
        return motorTalon.getVelocity().getValueAsDouble();
    }

    public void resetController(){
        motorTalonVelocityController = new VelocityVoltage(0);
    }

    private void configureMotor(){
        // Config motor
        motorTalonMotorConfiguration = new TalonFXConfiguration(); 
        motorTalonMotorConfiguration.MotorOutput.NeutralMode = this.motorTargetNeutralModeValue;
        motorTalonMotorConfiguration.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
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

    public void periodic(){
        SmartDashboard.putNumber("Spinner Speed", this.getSpeed());
        if(speedRpm != 0){
            motorTalon.setControl(this.motorTalonVelocityController.withVelocity(speedRpm/60));
        }
    }
}
