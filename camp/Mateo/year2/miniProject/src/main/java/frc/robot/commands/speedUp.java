package frc.robot.commands;

import frc.robot.subsystems.MySubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class speedUp extends Command {
    private final MySubsystem mySubsystem;

    public speedUp(MySubsystem subsystem) {
        mySubsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        @SuppressWarnings("unused")
        double speed = mySubsystem.getMotorSpeed();
        speed += 0.2;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
