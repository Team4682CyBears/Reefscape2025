package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeoMotorSubsystem;

public class WindCommand extends Command {
    NeoMotorSubsystem climberSubsystem;

    public WindCommand(NeoMotorSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setClimberSpeed(-0.4);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
