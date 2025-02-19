package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonMotorSubsystem;

public class UnwindCommand extends Command {
    TalonMotorSubsystem motorSubsystem;

    public UnwindCommand(TalonMotorSubsystem motorSubsystem) {
        this.motorSubsystem = motorSubsystem;
        
        addRequirements(motorSubsystem);
    }

    @Override
    public void initialize() {
        motorSubsystem.spinMotor(0.4);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
