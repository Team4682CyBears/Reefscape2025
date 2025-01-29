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

    Orchestra m_Orchestra = new Orchestra();
    TalonFX m_motor;
    

    /**
     * Constructs a MotorSubsystem object with the specified motor port.
     * 
     * @param motorPort the port number of the motor
     */
    public MotorSubsystem(int motorPort) {
        this.m_motor = new TalonFX(motorPort);
        this.m_Orchestra.addInstrument(m_motor);
        String path = Filesystem.getDeployDirectory().getAbsolutePath() + "track.chrp";
        System.out.println(path);
        StatusCode status = m_Orchestra.loadMusic(path);
        
        if (!status.isOK()) {
            System.out.println("Error: " + status.toString());
        }
    }

    public void playMusic() {
        m_Orchestra.play();
    }
 
    public void stopMusic() {
        m_Orchestra.stop();
    }
}