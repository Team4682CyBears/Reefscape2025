// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: OpenFunnelCommand.java
// Intent: Forms a command to open the funnel.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.SimpleNeoMotorSubsystem;

/**
 * Forms a command to open the funnel. 
 */
public class OpenFunnelCommand extends Command {
    SimpleNeoMotorSubsystem funnelSubsystem;
    
    private static final double openSpeed = Constants.funnelMotorSpeed;

    /**
     * Constructor for open funnel command. 
     * @param funnelSubsystem
     */
    public OpenFunnelCommand(SimpleNeoMotorSubsystem funnelSubsystem) {
        this.funnelSubsystem = funnelSubsystem;

        addRequirements(this.funnelSubsystem);
    }

    /**
     * Called once per tick when the command is running. 
     */
    @Override
    public void execute() {
        funnelSubsystem.setSpeed(openSpeed);
    }

    /**
     * Determines when the command should end.
     */
    @Override
    public void end(boolean interrupted) {
        this.funnelSubsystem.stopMotor();
    }

    /**
     * Returns true when the command should end. 
     */
    @Override
    public boolean isFinished() {
        // Using this command with an onTrue trigger. Command will be canceled once
        // the trigger is false. 
        return false; 
    }
}
