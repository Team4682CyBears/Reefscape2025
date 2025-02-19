package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonMotorSubsystem;


public class StopCommand extends Command {
    TalonMotorSubsystem motorSubsystem;

    public StopCommand(TalonMotorSubsystem motorSubsystem) {
        // Setup
        System.out.println("----public StopCommand(TalonMotorSubsystem motorSubsysstem) in StopCommand.java is running----");
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
        // The actual command. Stops the motor when no buttons are being pressed.
>>>>>>> 1520e6510ce2b170224525aa64c1afdc211a1e2d
        System.out.println("----public void initialize() in StopCommand.java is running----");
        motorSubsystem.motorStop(); // Calls on the motorStop command in TalonMotorSubsystem
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
