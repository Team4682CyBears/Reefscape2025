// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ClearAlgaeCommand.java
// Intent: Command to clear algae from end effector
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ClearAlgaeCommand extends Command {
    // The subsystem that controls the end effector mechanism
    private final EndEffectorSubsystem endEffector;

    /**
     * Creates a new ClearAlgaeCommand.
     * 
     * @param subsystem The EndEffectorSubsystem that this command will use
     */
    public ClearAlgaeCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(endEffector); // Ensure no other command uses this subsystem simultaneously
    }

    /**
     * Called repeatedly when this Command is scheduled to run.
     * Sets the end effector to the appropriate direction and speed for algae
     * clearing.
     */
    @Override
    public void execute() {
        // Configure end effector for algae clearing operation
        endEffector.setDirection(EndEffectorDirection.ALGAE);
        endEffector.setSpeed(EndEffectorSpeed.ALGAE);
    }

    /**
     * Called once when the command ends or is interrupted.
     * 
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the end effector when the command ends
        endEffector.stop();
    }

    /**
     * Returns false to run until interrupted, since algae clearing is a continuous
     * operation.
     * 
     * @return false to run continuously
     */
    @Override
    public boolean isFinished() {
        return false; // Run continuously until interrupted
    }
}
