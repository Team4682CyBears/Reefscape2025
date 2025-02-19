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
<<<<<<< HEAD
    }

    @Override
    public void execute(){
=======
        // The actual command. Sets speedRpm to -0.4 for the purpose of spinning the motor coutnerclockwise
>>>>>>> 1520e6510ce2b170224525aa64c1afdc211a1e2d
        System.out.println("----public void initialize() in WindCommand.java is running----");
        motorSubsystem.spinMotor(-0.4); // Calls on the spinMotor command in TalonMotorSubsystem and sets speedRpm to -0.4
    }

    @Override
    public boolean isFinished() {
        return false;  
    }
}
