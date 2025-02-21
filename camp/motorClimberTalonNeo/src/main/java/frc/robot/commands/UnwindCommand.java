package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.TalonMotorSubsystem;


public class UnwindCommand extends Command {
    ClimberSubsystem climberSubsystem;

    public UnwindCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setClimberSpeed(0.075);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
