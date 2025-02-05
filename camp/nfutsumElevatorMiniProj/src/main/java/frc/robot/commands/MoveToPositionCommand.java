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
        System.out.println("Moving elevator to position " + targetPosition);
    }

    public void execute() {
        if(targetPosition == ElevatorPositions.STOW){
            elevatorSubsystem.moveToPosition(Constants.stowHeight);
            System.out.println("Setting to stow");
        }else if(targetPosition == ElevatorPositions.L2){
            elevatorSubsystem.moveToPosition(Constants.L2Height);
            System.out.println("setting to L2");
        }else if(targetPosition == ElevatorPositions.L3){
            elevatorSubsystem.moveToPosition(Constants.L3Height);
            System.out.println("setting to L3");
        }else if(targetPosition == ElevatorPositions.L4){
            elevatorSubsystem.moveToPosition(Constants.L4Height);
            System.out.println("setting to L4");
        }
 
    }

    @Override
    public boolean isFinished(){
        return elevatorSubsystem.isAtTargetHeight();
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted){
            System.out.println("Command interrupted!");
        }
        else {
            System.out.println("Command ended!");
        }
    }
}
