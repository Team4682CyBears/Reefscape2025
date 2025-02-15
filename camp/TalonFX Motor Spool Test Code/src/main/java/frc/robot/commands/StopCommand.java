package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonMotorSubsystem;


public class StopCommand extends Command {
    TalonMotorSubsystem motorSubsystem;

    public StopCommand(TalonMotorSubsystem motorSubsystem) {
        System.out.println("----public StopCommand(TalonMotorSubsystem motorSubsysstem) in StopCommand.java is running----");
        this.motorSubsystem = motorSubsystem;
        
        addRequirements(motorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("----public void initialize() in StopCommand.java is running----");
        motorSubsystem.motorStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
