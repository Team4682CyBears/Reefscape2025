// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ClimberSubsystem.java
// Intent: Configures and runs our Neo motor for the climber
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.SparkBase.IdleMode;
//import com.revrobotics.spark.SparkBase.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;;

public class ClimberSubsystem extends SubsystemBase {
    // configurtition for the motor begins here
    private SparkMax climberMotor = null;

    // Value from 0.0 to 1.0 scaling the joystick input
    public static double maxClimberSpeed = 0.4;

    public ClimberSubsystem() {

        climberMotor = new SparkMax(Constants.climberMotorCanID, MotorType.kBrushless);

        // Configure motor to break when stopped
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // Apply config
        climberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * A method to set the climber speed
     * 
     * @param speed (between -1 and 1)
     */
    public void setClimberSpeed(double speed) {
        climberMotor.set(clamp(speed, -1.0, 1.0));
    }

    /**
     * A method to stop the climber subsystem
     */
    public void setAllStop() {
        climberMotor.stopMotor();
    }

    /**
     * Constrains a value between a minimum and maximum value.
     * 
     * @param x   The value to clamp
     * @param min The minimum value allowed
     * @param max The maximum value allowed
     * @return The clamped value, constrained between min and max
     */
    private double clamp(double x, double min, double max) {
        return Math.min(Math.max(x, min), max);
    }
}