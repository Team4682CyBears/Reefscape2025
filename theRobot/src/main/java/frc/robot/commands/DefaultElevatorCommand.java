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
    }

    /**
     * Is called once per tick while command is scheduled.
     */
    @Override
    public void execute() {
        if (joystickAxisSupplier.getAsDouble() > 0) {
            elevatorSubsystem.moveUp();
        } 
        else if (joystickAxisSupplier.getAsDouble() < 0) {
            elevatorSubsystem.moveDown();    
        }
    }

    /**
     * Is called when the command ends. 
     */
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
    }

    /**
     * Returns true if the command should end.
     */
    @Override
    public boolean isFinished(){
        return false; // default command keeps running until interrupted
    }
}