// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: StopEndEffectorCommand.java
// Intent: Default command to stop the end effector
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class StopEndEffectorCommand extends Command {
    // The subsystem that controls the end effector mechanism
    private final EndEffectorSubsystem endEffector;

    /**
     * Creates a new StopEndEffectorCommand.
     * 
     * @param subsystem The EndEffectorSubsystem that this command will use
     */
    public StopEndEffectorCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(endEffector);
    }

    /**
     * Called repeatedly when this Command is scheduled to run.
     * Immediately stops the end effector's motion.
     */
    @Override
    public void execute() {
        // Stop all end effector motion
        endEffector.stop();
    }
}
