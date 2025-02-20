package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeoMotorSubsystem;
import frc.robot.subsystems.TalonMotorSubsystem;


public class StopCommand extends Command {
    Object climberSubsystem;

    public StopCommand(Class<?> climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(climberSubsystem);
    }

    public StopCommand(NeoMotorSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setAllStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
