package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents a motor subsystem that controls a single motor.
 */
public class MotorSubsystem implements Subsystem {
    public TalonFX m_motor;
    

    /**
     * Constructs a MotorSubsystem object with the specified motor port.
     * 
     * @param motorPort the port number of the motor
     */
    public MotorSubsystem(int motorPort) {
        this.m_motor = new TalonFX(motorPort);
    }
}