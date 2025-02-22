// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reef - 2025
// File: public enum CorrectableEncoderRevNeoPlusDigitalIoPort.java
// Intent: Forms an interface that will help keep a motor encoder from drifting with use of external sensor.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class CorrectableEncoderPlusDigitalIoPort implements ICorrectableEncoderNaher {

    private TalonFX talonFXMotorEncoder = null;
    private DigitalInput dioStateDevice = null;
    private double encoderRotationAtSensorPosition = 0.0;
    private boolean lastState = false;
    private boolean motorEncoderPositionReset = false;

    /**
     * Concrete implementation of a pairing between motor encoder and DIO to coordinate resetting of 
     * encoder position based on state change of DIO
     * @param talonFXEncoder - the Talon motor encoder
     * @param stateDevice - the DIO state device
     * @param encoderRotationAtSensorPosition - the set point for the encoderRotation corresponding the sensor location. 
     * @param sensorInitialPositionRotation - the initial set point to use for the motors encoder
     */
    public CorrectableEncoderPlusDigitalIoPort(
        TalonFX talonFXEncoder,
        DigitalInput stateDevice,
        double encoderRotationAtSensorPosition,
        double sensorInitialPositionRotation) {

        this.talonFXMotorEncoder = talonFXEncoder;
        dioStateDevice = stateDevice;
        this.encoderRotationAtSensorPosition = encoderRotationAtSensorPosition;

        lastState = dioStateDevice.get();

        this.talonFXMotorEncoder.setPosition(sensorInitialPositionRotation);
        System.out.println("Initializing Correctable Encoder to " + sensorInitialPositionRotation + " rotations");
    }

    /**
     * A method to return the current motor encoder position.
     * @return a double representing the motor encoder Rotation
     */
    public Angle getCurrentEncoderPosition() {
        StatusSignal<Angle> position = this.talonFXMotorEncoder.getPosition();
        return position.getValue();
    }

    /**
     * A method intended to be called periodicially (as in 1x per 20 ms) that will be used to 
     * correct the motor encode (when necessary).
     */
    public void updateEncoderPosition() {
        boolean currentState = this.dioStateDevice.get();

        if(currentState != this.lastState) {
            this.talonFXMotorEncoder.setPosition(encoderRotationAtSensorPosition);
            System.out.println("Resetting encoder position to mag sensor height");
            this.lastState = currentState;
            motorEncoderPositionReset = true;
        }
    }

    /**
     * An  method that will be used to determine if the motor encoder position has ever been reset.
     * @return a boolean to describe if the motor encoder has ever been reset
     */
    public boolean getMotorEncoderEverReset() {
        return motorEncoderPositionReset;
    }

}