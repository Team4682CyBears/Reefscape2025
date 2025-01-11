package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.Constants;

public class MotorDefaultCommand extends Command {
    private MotorSubsystem motorSubsystem;
    private DoubleSupplier doubleSupplier;
    private boolean done = false;

    private static Timer timer = new Timer();

    /**
     * Constructs a MotorDefaultCommand.
     * 
     * @param motorSubsystem the motor subsystem to control
     * @param doubleSupplier double input for the motor speed
     */
    public MotorDefaultCommand(MotorSubsystem motorSubsystem, DoubleSupplier doubleSupplier) {
        this.motorSubsystem = motorSubsystem;
        this.doubleSupplier = doubleSupplier;
        addRequirements(motorSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
    }

    /**
     * Executes the command by setting the speed of the motor subsystem based on the
     * left Y direction of the controller.
     */
    @Override
    public void execute() {
        motorSubsystem.setSpeed(doubleSupplier.getAsDouble());
        if (timer.hasElapsed(Constants.motorRunTime)) {
            done = true;
        }
    }

    
    @Override
    public boolean isFinished() {
        return done;
    }
}
