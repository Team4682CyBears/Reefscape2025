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
import frc.robot.subsystems.SimpleNeoMotorSubsystem;

/**
 * Forms a default command to stop the climber. 
 */
public class DefaultClimberCommand extends Command {
    SimpleNeoMotorSubsystem climberSubsystem;
    DoubleSupplier speedSupplier;

    /**
     * Default command to stop the climber. 
     * @param climberSubsystem
     * @param speedSupplier
     */
    public DefaultClimberCommand(SimpleNeoMotorSubsystem climberSubsystem, DoubleSupplier speedSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.speedSupplier = speedSupplier;

        addRequirements(this.climberSubsystem);
    }

    /**
     * Called every tick when the command is running. 
     */
    @Override
    public void execute() {
        climberSubsystem.setSpeed(speedSupplier.getAsDouble());
    }

    /**
     * Called when the command ends
     */
    @Override
    public void end(boolean interrupted) {
        this.climberSubsystem.stopMotor();
    }

    /**
     * Determines whether the command should end.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
