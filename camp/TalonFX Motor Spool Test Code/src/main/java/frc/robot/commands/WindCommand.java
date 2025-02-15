package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonMotorSubsystem;

public class WindCommand extends Command {
    TalonMotorSubsystem motorSubsystem;

    public WindCommand(TalonMotorSubsystem motorSubsystem) {
        // Setup
        System.out.println("----public WindCommand(TalonMotorSubsystem motorSubsystem) in WindCommand.java is running----");
        this.motorSubsystem = motorSubsystem; // Creates a local instance of motorSubsystem
        
        addRequirements(motorSubsystem); // Adds motorSubsystem as a requirement
    }

    @Override
    public void initialize() {
        // The actual command. Sets speedRpm to -0.4 for the purpose of spinning the motor coutnerclockwise
        System.out.println("----public void initialize() in WindCommand.java is running----");
        motorSubsystem.spinMotor(-0.4); // Calls on the spinMotor command in TalonMotorSubsystem and sets speedRpm to -0.4
    }

    @Override
    public boolean isFinished() {
        return false;  
    }
}
