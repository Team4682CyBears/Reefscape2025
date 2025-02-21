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
import frc.robot.subsystems.FunnelSubsystem;

public class OpenFunnelCommand extends Command {
    FunnelSubsystem funnelSubsystem;
    
    private static final double openSpeed = 0.2; // TODO: Set to real value

    public OpenFunnelCommand(FunnelSubsystem funnelSubsystem) {
        this.funnelSubsystem = funnelSubsystem;

        addRequirements(this.funnelSubsystem);
    }

    @Override
    public void execute() {
        funnelSubsystem.setFunnelSpeed(openSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        this.funnelSubsystem.stopFunnel();
    }

    @Override
    public boolean isFinished() {
        return funnelSubsystem.isObjectDetected(); // TODO:
    }
}
