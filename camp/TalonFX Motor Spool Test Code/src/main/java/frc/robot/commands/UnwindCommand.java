package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonMotorSubsystem;

public class UnwindCommand extends Command {
    TalonMotorSubsystem motorSubsystem;

    public UnwindCommand(TalonMotorSubsystem motorSubsystem) {
        // Setup
        System.out.println("----public UnwindCommand(TalonMotorSubsystem motorSubsystem) in UnwindCommand.java is running----");
        this.motorSubsystem = motorSubsystem; // Creates a local instance of motorSubsystem
        
        addRequirements(motorSubsystem); // Adds motorSubsystem as a requirement
    }

    @Override
    public void initialize() {
        // The actual command. Sets speedRpm to 0.4 for the purpose of making the motor spin clockwise
        System.out.println("----public void initialize() in UnwindCommand.java is running----");
        motorSubsystem.spinMotor(0.4); // Calls on the spinMotor command in TalonMotorSubsystem and sets speedRpm to 0.4
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
