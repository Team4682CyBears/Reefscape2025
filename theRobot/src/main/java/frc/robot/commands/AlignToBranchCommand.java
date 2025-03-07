// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: AlignWithBranchCommand.java
// Intent: Forms a command to align with branches using ToFs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.AlignToBranchSide;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.BranchDetectorSubsystem;
import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
// Network Tables
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;

/**
 * Class to form a command to align with branch using ToFs
 */
public class AlignToBranchCommand extends Command{
    private DrivetrainSubsystem drivetrain;
    private BranchDetectorSubsystem branchDetector;
    private Supplier<AlignToBranchSide> alignSideSupplier;
    private boolean done = false;
    private Timer timer = new Timer();
    private double durationSeconds = 2;
    private double yVelocity = .4;
    private ChassisSpeeds chassisSpeeds;
    
    /**
     * Constructor to make the robot aling with a branch
     * @param drivetrainsubsystem - the drivetrain subsystems
     * @param branchDetector - the branch detector subsystem
     * @param alignSideSupplier - a supplier of an enum that tells us if we want to align right or left
     */
    public AlignToBranchCommand(DrivetrainSubsystem drivetrainSubsystem, BranchDetectorSubsystem branchDetector, Supplier<AlignToBranchSide> alignSideeSupplier){
        this.drivetrain = drivetrainSubsystem;
        this.branchDetector = branchDetector;
        this.alignSideSupplier = alignSideeSupplier;

        // explicitly not requiring the branch detector here because we are using it in a read-only capacity.
        addRequirements(drivetrainSubsystem);
    }
    

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        done = false;

        if(alignSideSupplier.get() == AlignToBranchSide.RIGHT){
            chassisSpeeds = new ChassisSpeeds(0.0, -yVelocity, 0.0);
        }
        else{
            chassisSpeeds = new ChassisSpeeds(0.0, yVelocity, 0.0);
        }

        // Network Tables 
        //NetworkTableInstance toFPlotting = NetworkTableInstance.getDefault();
        //NetworkTable table = toFPlotting.getTable("datatable");
        
        // get a topic from a NetworkTableInstance
        // the topic name in this case is the full name
        //DoubleTopic dblTopic = toFPlotting.getDoubleTopic("/datatable/X");

        // get a topic from a NetworkTable
        // the topic name in this case is the name within the table;
        // this line and the one above reference the same topic
        //DoubleTopic dblTopic = table.getDoubleTopic("X");

        // get a type-specific topic from a generic Topic
        //Topic genericTopic = toFPlotting.getTopic("/datatable/X");
        //DoubleTopic dblTopic = new DoubleTopic(genericTopic);



    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        


        if(branchDetector.isBranchDetected() || timer.hasElapsed(this.durationSeconds)){
            done = true;
        }
        else {
            drivetrain.driveRobotCentric(chassisSpeeds);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        drivetrain.driveFieldCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
        if(interrupted)
        {
        done = true;      
        }
    }

    // Returns if the command is done.
    @Override
    public boolean isFinished(){
        return done;
    }
}
