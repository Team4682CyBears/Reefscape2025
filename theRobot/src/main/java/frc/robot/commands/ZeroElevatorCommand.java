// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ZeroElevatorCommand.java
// Intent: Command to run the elevator down until the limit switch is pressed,
//         then re-zero the elevator position.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.control.Constants;

/**
 * Command to run the elevator down until the limit switch is pressed,
 * then re-zero the elevator position.
 */
public class ZeroElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private static final double TIMEOUT_SECONDS = 3.0;
    private double startTime;

    /**
     * Constructor for ZeroElevatorCommand.
     * 
     * @param elevatorSubsystem The elevator subsystem to control.
     */
    public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(this.elevatorSubsystem);
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /**
     * Called once per tick when the command is running.
     */
    @Override
    public void execute() {
        elevatorSubsystem.moveDown();
    }

    @Override
    public boolean isFinished() {
        double elapsedTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime;
        return elevatorSubsystem.isLimitSwitchPressed() || elapsedTime >= TIMEOUT_SECONDS;
    }

    /**
     * Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();

        if (!interrupted) {
            elevatorSubsystem.setCurrentHeight(Constants.zeroElevatorHeight);;
            System.out.println("--------------Elevator zeroed successfully");
        } else {
            System.out.println("--------------ZeroElevatorCommand interrupted");
        }
    }
}