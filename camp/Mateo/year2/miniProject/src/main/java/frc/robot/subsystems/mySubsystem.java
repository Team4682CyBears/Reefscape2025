package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MySubsystem extends SubsystemBase {
    private double MotorSpeed = 0.0;

    public MySubsystem() {

    }

    public double getMotorSpeed() {
        return MotorSpeed;
    }

    public void setMotorSpeed(double speed) {
        MotorSpeed = speed;
    }
}