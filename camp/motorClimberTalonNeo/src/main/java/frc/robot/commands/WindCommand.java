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
        climberSubsystem.setClimberSpeed(Constants.windSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
