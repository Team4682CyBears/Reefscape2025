package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonMotorSubsystem;


public class StopCommand extends Command {
    TalonMotorSubsystem motorSubsystem;

    public StopCommand(TalonMotorSubsystem motorSubsystem) {
        this.motorSubsystem = motorSubsystem;
        
        addRequirements(motorSubsystem);
    }

    @Override
    public void initialize() {
        motorSubsystem.motorStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
