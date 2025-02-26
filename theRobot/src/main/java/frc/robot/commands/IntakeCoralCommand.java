// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: IntakeCoralCommand.java
// Intent: Command to intake coral
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.subsystems.EndEffectorSubsystem;

public class IntakeCoralCommand extends Command {
    // The subsystem that controls the end effector mechanism
    private final EndEffectorSubsystem endEffector;
    
    private boolean seenPiece = false;

    private Timer timeoutTimer = new Timer();

    private final double timeoutSeconds = 5.0;

    /**
     * Creates a new IntakeCoralCommand.
     * 
     * @param subsystem The EndEffectorSubsystem that this command will use
     */
    public IntakeCoralCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(endEffector);
    }

    /**
     * Called when the command is initially scheduled.
     * Checks if a branch is already detected and sets initial state.
     */
    @Override
    public void initialize() {
        timeoutTimer.reset();
        timeoutTimer.start();
    }

    /**
     * Called repeatedly when this Command is scheduled to run.
     * Controls the end effector for coral handoff until a branch is detected.
     */
    @Override
    public void execute() {
        if (!seenPiece && timeoutTimer.hasElapsed(timeoutSeconds)) {
            // If we don't see a piece after 5 seconds stop the motor
            this.end(false);
        }

        // Check for branch detection and stop if detected
        if (seenPiece && !endEffector.isCoralDetected()) {
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
        return seenPiece && !endEffector.isCoralDetected();
    }
}
