// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ScoreCoralCommand.java
// Intent: Command to score coral on the reef
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ScoreCoralCommand extends Command {
    // The subsystem that controls the end effector mechanism
    private final EndEffectorSubsystem endEffector;

    /**
     * Creates a new ScoreCoralCommand.
     * 
     * @param subsystem The EndEffectorSubsystem that this command will use
     */
    public ScoreCoralCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(endEffector);
    }

    /**
     * Called repeatedly when this Command is scheduled to run.
     * Sets the end effector to the appropriate direction and speed for coral
     * scoring.
     */
    @Override
    public void execute() { // Configure end effector for coral scoring operation
        endEffector.setDirection(EndEffectorDirection.CORAL);
        endEffector.setSpeed(EndEffectorSpeed.SCORING);
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
     * Returns whether the command should end.
     * 
     * @return false until scoring detection is implemented
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
