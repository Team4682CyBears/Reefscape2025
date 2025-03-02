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
import frc.robot.common.ElevatorPositions;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.control.ElevatorHeightState;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class DefaultEndEffectorCommand extends Command {
    // The subsystem that controls the end effector mechanism
    private final SubsystemCollection subsystems;

    private boolean seenPiece = false;

    /**
     * Creates a new IntakeCoralCommand.
     * 
     * @param subsystem The EndEffectorSubsystem that this command will use
     */
    public DefaultEndEffectorCommand(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        addRequirements(subsystems.getEndEffectorSubsystem());
    }

    /**
     * Called when the command is initially scheduled.
     * Checks if a branch is already detected and sets initial state.
     */
    @Override
    public void initialize() {
        seenPiece = false;
    }

    /**
     * Called repeatedly when this Command is scheduled to run.
     * Controls the end effector for coral handoff until a branch is detected.
     */
    @Override
    public void execute() {
        // Check for branch detection and stop if detected
        if(subsystems.getEndEffectorSubsystem().isCoralDetected() && subsystems.getElevatorHeightState().gElevatorPosition() == ElevatorPositions.STOW){
            subsystems.getEndEffectorSubsystem().setDirection(EndEffectorDirection.CORAL);
            subsystems.getEndEffectorSubsystem().setSpeed(EndEffectorSpeed.HANDOFF);
        }
        else {
            subsystems.getEndEffectorSubsystem().stop();
        }
    }

    /**
     * Called once when the command ends or is interrupted.
     * 
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        subsystems.getEndEffectorSubsystem().stop();
    }

    /**
     * Returns true when the command should end.
     * 
     * @return true if a branch has been detected and handled
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
