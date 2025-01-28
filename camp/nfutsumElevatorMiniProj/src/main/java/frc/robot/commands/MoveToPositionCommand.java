package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.common.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class MoveToPositionCommand extends Command{

    private final ElevatorSubsystem elevatorSubsystem;
    private final ElevatorPositions targetPosition;
    
    public MoveToPositionCommand(ElevatorSubsystem sub, ElevatorPositions target){
        this.elevatorSubsystem = sub;
        this.targetPosition = target;
        addRequirements(sub);

    }

    public void initialize() {
        elevatorSubsystem.moveToPosition(Constants.stowHeight);
    }

    public void execute() {
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

    public boolean isFinished(){
        if(elevatorSubsystem.isAtTargetHeight()){
            return true;
        }
        return false;
    }
}
