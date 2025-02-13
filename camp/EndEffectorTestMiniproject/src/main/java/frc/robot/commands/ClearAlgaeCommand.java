package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ClearAlgaeCommand extends Command {
    private final EndEffectorSubsystem endEffector;

    public ClearAlgaeCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        endEffector.setDirection(EndEffectorDirection.ALGAE);
        endEffector.setSpeed(EndEffectorSpeed.ALGAE);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.stop();
    }

    @Override
    public boolean isFinished() {
        // TODO: Detect when command is finished
        return false;
    }
}
