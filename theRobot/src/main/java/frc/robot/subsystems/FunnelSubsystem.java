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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ToFSensor;
import frc.robot.control.Constants;;

public class FunnelSubsystem extends SubsystemBase {
    // configurtition for the motor begins here
    private SparkMax funnelMotor = null;
    private ToFSensor funnelToF = null;

    private double tofDetectionThresholdInches = 20; // TODO: Set to real value

    public FunnelSubsystem() {
        funnelMotor = new SparkMax(Constants.funnelMotorCanID, MotorType.kBrushless);
        funnelToF = new ToFSensor(Constants.funnelTofCanID);

        // Configure motor to break when stopped
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        // Apply config
        funnelMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * A method to set the funnel speed
     * 
     * @param speed (between -1 and 1)
     */
    public void setFunnelSpeed(double speed) {
        funnelMotor.set(clamp(speed, -1.0, 1.0));
    }

    /**
     * A method to stop the funnel subsystem
     */
    public void stopFunnel() {
        funnelMotor.stopMotor();
    }

    /**
     * Determines if an object is detected by the ToF sensor.
     * 
     * @return true if an object is detected within the threshold distance, false
     *         otherwise
     */
    public boolean isObjectDetected() {
        return funnelToF.getRangeInches() <= tofDetectionThresholdInches;
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