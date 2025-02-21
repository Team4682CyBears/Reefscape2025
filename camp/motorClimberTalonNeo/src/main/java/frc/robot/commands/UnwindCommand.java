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
<<<<<<< HEAD
        climberSubsystem.setClimberSpeed(0.075);
=======
        climberSubsystem.setClimberSpeed(Constants.unwindSpeed);
>>>>>>> 346790b2601d4e252be7e191f3e69f5d6dcbafe8
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
