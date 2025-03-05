// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: IntakeCoralCommand.java
// Intent: Defualt command for the end effector 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.ElevatorPositions;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.control.SubsystemCollection;

/**
 * Forms a class that will run the end effector when we are in stow and there is a piece
 */
public class DefaultEndEffectorCommand extends Command {
    private final SubsystemCollection subsystems;

    /**
     * Creates a new IntakeCoralCommand.
     * 
     * @param subsystem The subsystem collection
     */
    public DefaultEndEffectorCommand(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        addRequirements(subsystems.getEndEffectorSubsystem());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(subsystems.getEndEffectorSubsystem().isCoralDetected() && subsystems.getElevatorHeightState().gElevatorPosition() == ElevatorPositions.STOW){
            subsystems.getEndEffectorSubsystem().setDirection(EndEffectorDirection.CORAL);
            subsystems.getEndEffectorSubsystem().setSpeed(EndEffectorSpeed.HANDOFF);
        }
        else {
            subsystems.getEndEffectorSubsystem().stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystems.getEndEffectorSubsystem().stop();
    }
}
