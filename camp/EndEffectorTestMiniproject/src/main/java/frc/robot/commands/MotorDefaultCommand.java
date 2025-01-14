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

    /**
     * Initializes the command by resetting and starting the timer, setting the done
     * flag to false,
     * and setting the motor speed using the provided double supplier.
     */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
        motorSubsystem.setSpeed(doubleSupplier.getAsDouble());
    }

    /**
     * Executes the command by setting the speed of the motor subsystem based on the
     * left Y direction of the controller.
     */
    @Override
    public void execute() {
        if (timer.hasElapsed(Constants.motorRunTime)) {
            done = true;
        }
    }

    /**
     * Checks if the command has finished execution.
     *
     * @return true if the command is done, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return done;
    }

    /**
     * This method is called when the command ends or is interrupted.
     * It stops the motor subsystem.
     *
     * @param interrupted true if the command was interrupted, false if it ended
     *                    normally
     */
    @Override
    public void end(boolean interrupted) {
        motorSubsystem.stop();
    }
}
