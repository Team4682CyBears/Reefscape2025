package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonSubsystem;

public class SpeedDownCommand extends Command{
    private TalonSubsystem talon;

    public SpeedDownCommand(TalonSubsystem talon){
        this.talon = talon;
        addRequirements(talon);
    }

    @Override
    public void execute() {
        talon.decreaseSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}