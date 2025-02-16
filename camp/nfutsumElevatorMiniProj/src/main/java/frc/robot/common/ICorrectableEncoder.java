// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: public enum CorrectableEncoder.java
// Intent: Forms an interface that will help keep a motor encoder from drifting with use of external sensor.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import edu.wpi.first.units.measure.Angle;

public interface ICorrectableEncoder {
    
    /**
     * An interface method that will be used to return the current motor encoder position.
     * @return a double representing the motor encoder rotations
     */
    public Angle getCurrentEncoderPosition();

    /**
     * An interface method that needs to be call in periodic to correct the sensor position (when necessary)
     * @return nothing
     */
    public void updateEncoderPosition();

    /**
     * An interface method that will be used to determine if the motor encoder position has ever been reset.
     * @return a boolean to describe if the motor encoder has ever been reset
     */
    public boolean getMotorEncoderEverReset();
}