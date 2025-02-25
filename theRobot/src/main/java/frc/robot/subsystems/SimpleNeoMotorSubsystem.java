// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ClimberSubsystem.java
// Intent: A simple subsystem that uses a Neo motor. Can be configured to run multiple subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.MotorUtils;

/**
 * Forms a class for a simple neo motor subsystem. 
 * Can be used in multiple places.
 * Supports a notion of a max speed, and setting the motor to the desired speed (up to max speed), 
 * and stopping the motor. 
 */
public class SimpleNeoMotorSubsystem extends SubsystemBase {
    // configurtition for the motor begins here
    private SparkMax motor = null;
    private double maxSpeed;

    /**
     * Constructor for simpleNeoMotorSubsystem. 
     * @param canID the can ID of the motor
     * @param maxSpeed the max speed of the motor. 
     */
    public SimpleNeoMotorSubsystem(int canID, double maxSpeed) {
        this.maxSpeed = maxSpeed;
        motor = new SparkMax(canID, MotorType.kBrushless);

        // Configure motor to break when stopped
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // Apply config
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * A method to return the max speed
     * @return max speed
     */
    public double getMaxSpeed(){
        return this.maxSpeed;
    }

    /**
     * A method to set the climber speed
     * 
     * @param speed (between -1 and 1)
     */
    public void setSpeed(double speed) {
        motor.set(MotorUtils.clamp(speed, -maxSpeed, maxSpeed));
    }

    /**
     * A method to stop the climber subsystem
     */
    public void stopMotor() {
        motor.stopMotor();
    }
}