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
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/**
 * Forms a class that will run the end effector when we are in stow and there is a piece
 */
public class DefaultEndEffectorCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;


    /**
     * Creates a new IntakeCoralCommand.
     * 
     * @param subsystem The subsystem collection
     */
    public DefaultEndEffectorCommand(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(endEffectorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(endEffectorSubsystem.isCoralDetected() && elevatorSubsystem.isAtStow()){
            endEffectorSubsystem.setDirection(EndEffectorDirection.CORAL);
            endEffectorSubsystem.setSpeed(EndEffectorSpeed.HANDOFF);
        }
        else {
            endEffectorSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.stop();
    }
}
