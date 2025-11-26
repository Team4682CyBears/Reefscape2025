package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonSubsystem;

public class StopTalonCommand extends Command{
    private TalonSubsystem talon;

    public StopTalonCommand(TalonSubsystem talon) {
        this.talon = talon;
        addRequirements(talon);
    }

    @Override
    public void execute() {
        talon.stopTalon();
    }

    @Override
    public boolean isFinished() {
        return(false);
    }
}
