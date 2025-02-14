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
    private Timer tofDetectionTimer = new Timer();

    public HandoffCoralCommand(EndEffectorSubsystem subsystem) {
        endEffector = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (!endEffector.isBranchDetected()) {
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
        // TODO
        if (!Constants.doubleTOF) {
            if (!endEffector.isBranchDetected()) {
                tofDetectionTimer.start();
            } else {
                tofDetectionTimer.reset();
            }
        }

        if ((Constants.doubleTOF && !endEffector.isBranchDetected())
                || (!Constants.doubleTOF && tofDetectionTimer.hasElapsed(0.05))) {
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
