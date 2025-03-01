// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: DefaultElevatorCommand.java
// Intent: Forms a command to control the elevator with a joystick.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Forms a default command for the elevator subsystem to be controlled by a joystick
 */
public class DefaultElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final DoubleSupplier joystickAxisSupplier;

    /**
     * Constructor for elevator subsystem.
     * @param elevatorSubsystem
     * @param joystickAxisSupplier
     */
    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem,
                               DoubleSupplier joystickAxisSupplier) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystickAxisSupplier = joystickAxisSupplier;
        addRequirements(elevatorSubsystem);
        System.out.println("--------------Elevator default command initialized");
    }

    /**
     * Is called once per tick while command is scheduled.
     */
    @Override
    public void execute() {
        double joystickValue = joystickAxisSupplier.getAsDouble();
        // System.out.println("----------------Elevator joystick value: " + joystickValue );
        if(joystickValue > 0) {
            elevatorSubsystem.moveUp();
        } 
        else if (joystickValue < 0) {
            elevatorSubsystem.moveDown();    
        } 
        else {
            elevatorSubsystem.stopElevator();
        }
    }

    /**
     * Is called when the command ends. 
     */
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
        System.out.println("-----------------Default command ended. Stopping Elevator");
    }

    /**
     * Returns true if the command should end.
     */
    @Override
    public boolean isFinished(){
        return false; // default command keeps running until interrupted
    }
}