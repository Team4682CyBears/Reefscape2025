// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: main robot body
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonSubsystem;

/**
 * This class calls the stopTalon() method in TalonSubsystem. It
 * sets the speed of the TalonFX motor to 0 and is bound to the
 * B button on an Xbox controller.
 */
public class StopTalonCommand extends Command{
    private TalonSubsystem talon; // Declaration of the TalonSubsystem
    
    /** This method initializes the local instance of talon and adds
     * it to the list of requirements.
     */
    public StopTalonCommand(TalonSubsystem talon) {
        this.talon = talon; // Initializes this instance of talon to the declared TalonSubsystem
        addRequirements(talon); // Adds this instance of the subsystem to the list of requirements
    }

    /**
     * This method executes the stopTalon() method in
     * TalonSubsystem.
     */
    @Override
    public void execute() {
        // Executes the stopTalon command
        talon.stopTalon();
    }

    @Override
    public boolean isFinished() {
        return(false);
    }
}
