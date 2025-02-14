package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.AlignToBranchSide;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChangeAlignWithBranchSideComand extends Command{
    private DrivetrainSubsystem drivetrain;
    private AlignToBranchSide alignSide;
    
    public ChangeAlignWithBranchSideComand(DrivetrainSubsystem drivetrain, AlignToBranchSide alignSide){
        this.drivetrain = drivetrain;
        this.alignSide = alignSide;
    }

    @Override
    public void initialize(){
        drivetrain.setAlignToBranchSide(alignSide);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
