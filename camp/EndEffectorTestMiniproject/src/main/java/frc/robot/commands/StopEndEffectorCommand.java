package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class StopEndEffectorCommand extends Command {
    private final EndEffectorSubsystem endEffector;

    public StopEndEffectorCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        endEffector.stop();
    }
}
