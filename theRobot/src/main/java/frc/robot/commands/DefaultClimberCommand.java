// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: DefaultClimberCommand.java
// Intent: Forms a command to control the climber.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends Command {
    ClimberSubsystem climberSubsystem;
    DoubleSupplier speedSupplier;

    public DefaultClimberCommand(ClimberSubsystem climberSubsystem, DoubleSupplier speedSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.speedSupplier = speedSupplier;

        addRequirements(this.climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.setClimberSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        this.climberSubsystem.setAllStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
