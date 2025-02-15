package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonMotorSubsystem;

public class UnwindCommand extends Command {
    TalonMotorSubsystem motorSubsystem;

    public UnwindCommand(TalonMotorSubsystem motorSubsystem) {
        System.out.println("----public UnwindCommand(TalonMotorSubsystem motorSubsystem) in UnwindCommand.java is running----");
        this.motorSubsystem = motorSubsystem;
        
        addRequirements(motorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        System.out.println("----public void initialize() in UnwindCommand.java is running----");
        motorSubsystem.spinMotor(0.4);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
