// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: MoveToPositionCommand.java
// Intent: Moves robot elevtor to preset height
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// namespace to contain class
package frc.robot.commands;

// import local classes
import frc.robot.Constants;
import frc.robot.common.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

// import wpilib classes
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPositionCommand extends Command{

    // create one assignment to classes instances // not initialized yet
    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorPositions targetPosition;
   
    // constructor
    public MoveToPositionCommand(ElevatorSubsystem sub, ElevatorPositions target){

        // initlialize class instances created above
        this.elevatorSubsystem = sub;
        this.targetPosition = target;

        // Use addRequirements() here to declare subsystem dependencies.
        //
        // Each command should declare any subsystems it controls as requirements. 
        // This backs the scheduler’s resource management mechanism, ensuring that 
        // no more than one command requires a given subsystem at the same time.
        // 
        // https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
        addRequirements(sub);
    }

    // command run when initially called
    public void initialize() {
        System.out.println("Moving elevator to position " + targetPosition);
    }

    // command called periodically, every 20ms by default
    public void execute() {

        // TODO AG - This should be a switch statement
        //         - Could just be in initialize, rather than rerun every period?
        // based on the selected target position, move the robot to the preset position
        if(targetPosition == ElevatorPositions.STOW){
            elevatorSubsystem.moveToPosition(Constants.stowHeight);
        }else if(targetPosition == ElevatorPositions.L2){
            elevatorSubsystem.moveToPosition(Constants.L2Height);
        }else if(targetPosition == ElevatorPositions.L3){
            elevatorSubsystem.moveToPosition(Constants.L3Height);
        }else if(targetPosition == ElevatorPositions.L4){
            elevatorSubsystem.moveToPosition(Constants.L4Height);
        }
    }


    // check if elevator at target height, return true if true (ends command)
    @Override
    public boolean isFinished(){
        return elevatorSubsystem.isAtTargetHeight();
    }

    // command run when command is over
    @Override
    public void end(boolean interrupted){

        // print how command ended
        if (interrupted){
            System.out.println("Command interrupted!");
        }
        else {
            System.out.println("Command ended!");
        }
    }
}
