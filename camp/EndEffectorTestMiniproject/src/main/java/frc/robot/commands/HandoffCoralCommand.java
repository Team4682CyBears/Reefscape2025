package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.subsystems.EndEffectorSubsystem;

public class HandoffCoralCommand extends Command {
    private final EndEffectorSubsystem endEffector;

    private boolean done = false;

    public HandoffCoralCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (endEffector.isBranchDetected()) {
            done = true;
        } else {
            done = false;
        }
        tofDetectionTimer.reset();
        tofDetectionTimer.stop();
    }

    @Override
    public void execute() {
        if (done) {
            return;
        }

        if (endEffector.isBranchDetected()) {
            System.out.println("STOPPING MOTOR");
            done = true;
            endEffector.stop();
            return;
        }
        endEffector.setDirection(EndEffectorDirection.CORAL);
        endEffector.setSpeed(EndEffectorSpeed.HANDOFF);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.stop();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
