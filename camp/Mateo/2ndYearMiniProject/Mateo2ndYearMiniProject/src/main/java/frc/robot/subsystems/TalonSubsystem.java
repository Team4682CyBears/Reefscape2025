// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: main robot body
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is the subsystem of the TalonFX motor. It contains 
 * the configuration for the TalonFX and all of the methods
 * used to change speed and run the motor.
 */

public class TalonSubsystem extends SubsystemBase { // The subsystem of the TalonFX motor that is being used
    public double speed = 0; // Initializes speed as 0
    private TalonFX talon = new TalonFX(Constants.talonID); // Creates the local instance of the TalonFX motor
    private DutyCycleOut tDutyCycle = new DutyCycleOut(0.0); // Creates the subsystem's DutyCycleOut method
    private final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;

    /**
     * This method decreases the speed of the TalonFX motor
     * by the RATEOFCHANGE constant (0.02) in Constants.
     * 
     * @return a message signalling that the
     * SpeedDownCommand is being run.
     */
    public void decreaseSpeed() {
        System.out.println("!!!!!!!!!!DECREASE SPEED!!!!!!!!!!!!"); // Prints to show what command is being run
        speed -= Constants.rateOfChange; // Decreases speed by the rate of change
    }

    /**
     * This method increases the speed of the TalonFX motor
     * by the RATEOFCHANGE constant (0.02) in Constants.
     * 
     * @return a message signalling that the
     * SpeedUpCommand is being run.
     */
    public void increaseSpeed() {
        System.out.println("!!!!!!!!!!INCREASE SPEED!!!!!!!!!!!!"); // Prints to show what command is being run
        speed += Constants.rateOfChange; // Increases speed by the rate of change
    }

    /**
     * This method is called once per tick. It calls the
     * talon.stopMotor method if the speed has been set
     * to 0. If not, it clamps the speed of the TalonFX
     * motor between -1.0 and 1.0 and sets the actual
     * speed of the talon to the speed variable.
     */
    public void periodic() {
        if (speed == 0) {
            // Stop the motor
            talon.stopMotor();
        } else {
            // clamp the speed to +/-1 as required by the motor limits
            tDutyCycle.withOutput(MathUtil.clamp(speed, -1.0, 1.0));
            talon.setControl(tDutyCycle);
        }
    }

    /**
     * This method sets the speed of the TalonFX motor
     * to 0.
     * 
     * @return a message signalling that the
     * StopTalonCommand is being run.
     */
    public void stopTalon() {
        System.out.println("!!!!!!!!!!STOP TALON!!!!!!!!!!!!!!!!"); // Prints to show what command is being run
        speed = 0; // Sets speed to 0
    }

    /**
     * This method calls the configureMotor() method in
     * order to configure the TalonFX.
     */
    public TalonSubsystem() {
        configureMotor(); // Configures the motor
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Configuration of voltage
        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.Voltage.SupplyVoltageTimeConstant = 0.02;

        // Configuration of current limits
        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = motorOutputInverted;

        StatusCode response = talon.getConfigurator().apply(config);
        if (!response.isOK()) {
            // Notifies the user if configuration has failed
            System.out.println(
                    "TalonFX ID " + talon.getDeviceID() + " failed config with error " + response.toString());
        }
    }
}
