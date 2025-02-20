package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NeoMotorSubsystem;
//import frc.robot.subsystems.TalonMotorSubsystem;

public class UnwindCommand extends Command {
    NeoMotorSubsystem climberSubsystem;

    public UnwindCommand(NeoMotorSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setClimberSpeed(0.4);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
