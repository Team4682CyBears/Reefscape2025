package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonSubsystem;

public class SpeedUpCommand extends Command{
    private TalonSubsystem talon;

    public SpeedUpCommand(TalonSubsystem talon){
        this.talon = talon;
        addRequirements(talon);
    }

    @Override
    public void execute() {
        talon.increaseSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}