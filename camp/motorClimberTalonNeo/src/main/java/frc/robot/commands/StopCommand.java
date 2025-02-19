package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.TalonMotorSubsystem;


public class StopCommand extends Command {
    ClimberSubsystem climberSubsystem;

    public StopCommand(ClimberSubsystem climberSubsystem) {
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
