// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: HandoffCoralCommand.java
// Intent: Command to handoff coral
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.subsystems.EndEffectorSubsystem;

public class HandoffCoralCommand extends Command {
    // The subsystem that controls the end effector mechanism
    private final EndEffectorSubsystem endEffector;
    
    // Tracks whether the command has completed its operation
    private boolean done = false;

    /**
     * Creates a new HandoffCoralCommand.
     * 
     * @param subsystem The EndEffectorSubsystem that this command will use
     */
    public HandoffCoralCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(endEffector);
    }

    /**
     * Called when the command is initially scheduled.
     * Checks if a branch is already detected and sets initial state.
     */
    @Override
    public void initialize() {
        // If branch is already detected, mark as done immediately
        if (endEffector.isCoralDetected()) {
            done = true;
        } else {
            done = false;
        }
    }

    /**
     * Called repeatedly when this Command is scheduled to run.
     * Controls the end effector for coral handoff until a branch is detected.
     */
    @Override
    public void execute() {
        // Skip execution if already done
        if (done) {
            return;
        }

        // Check for branch detection and stop if detected
        if (endEffector.isCoralDetected()) {
            System.out.println("STOPPING MOTOR");
            done = true;
            endEffector.stop();
            return;
        }

        // Continue running end effector for coral handoff
        endEffector.setDirection(EndEffectorDirection.CORAL);
        endEffector.setSpeed(EndEffectorSpeed.HANDOFF);
    }

    /**
     * Called once when the command ends or is interrupted.
     * 
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        endEffector.stop();
    }

    /**
     * Returns true when the command should end.
     * 
     * @return true if a branch has been detected and handled
     */
    @Override
    public boolean isFinished() {
        return done;
    }
}
