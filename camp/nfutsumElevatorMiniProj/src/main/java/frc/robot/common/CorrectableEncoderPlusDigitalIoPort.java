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

public class CorrectableEncoderPlusDigitalIoPort implements ICorrectableEncoder {

    private TalonFX talonFXMotorEncoder = null;
    private DigitalInput dioStateDevice = null;
    private double encoderRotationAtStateChangeSetPoint = 0.0;
    private boolean lastState = false;
    private boolean motorEncoderPositionReset = false;

    /**
     * Concrete implementation of a pairing between motor encoder and DIO to coordinate resetting of 
     * encoder position based on state change of DIO
     * @param revNeoEncoder - the REV Neo motor encoder
     * @param stateDevice - the DIO state device
     * @param encoderRotationAtStateChange - the set point to use for the motors encoder when the state
     * changes between the current and previous calls into getCurrentEncoderPosition
     * @param sensorTriggeredEncoderInitialPositionRotation - the initial set point to use for the motors
     * encoder when sensor IS triggered to start 
     * @param sensorNotTriggeredEncoderInitialPositionRotation - the initial set point to use for the motors
     * encoder when sensor is NOT triggered to start 
     */
    public CorrectableEncoderPlusDigitalIoPort(
        TalonFX talonFXEncoder,
        DigitalInput stateDevice,
        double encoderRotationAtStateChange,
        double sensorTriggeredEncoderInitialPositionRotation,
        double sensorNotTriggeredEncoderInitialPositionRotation) {

        talonFXMotorEncoder = talonFXEncoder;
        dioStateDevice = stateDevice;
        encoderRotationAtStateChangeSetPoint = encoderRotationAtStateChange;

        lastState = dioStateDevice.get();

        // dio state of false is 'triggered' (as in LED is illuminated for 2023 sensors)
        if(lastState == false) {
            this.talonFXMotorEncoder.setPosition(sensorTriggeredEncoderInitialPositionRotation);
        }
        else {
            this.talonFXMotorEncoder.setPosition(sensorNotTriggeredEncoderInitialPositionRotation);
        }
    }

    /**
     * An method intended to be called periodically (as in 1x per 20 ms) that will be used to both
     * correct the motors encoder (when necessary) and also return the current motor encoder position.
     * @return a double representing the motor encoder Rotation
     */
    public StatusSignal<Angle> getCurrentEncoderPosition() {

        boolean currentState = this.dioStateDevice.get();

        if(currentState != this.lastState) {
            this.talonFXMotorEncoder.setPosition(encoderRotationAtStateChangeSetPoint);
            this.lastState = currentState;
            motorEncoderPositionReset = true;
        }

        return this.talonFXMotorEncoder.getPosition();
    }

    /**
     * An  method that will be used to determine if the motor encoder position has ever been reset.
     * @return a boolean to describe if the motor encoder has ever been reset
     */
    public boolean getMotorEncoderEverReset() {
        return motorEncoderPositionReset;
    }

}