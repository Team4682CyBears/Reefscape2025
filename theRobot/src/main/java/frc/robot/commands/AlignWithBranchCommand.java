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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.common.AlignToBranchSide;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import java.util.function.Supplier;


/**
 * Class to form a command to align with branch using ToFs
 */
public class AlignWithBranchCommand extends Command{
    private DrivetrainSubsystem drivetrain;
    private EndEffectorSubsystem endEffector;
    private Supplier<AlignToBranchSide> alignSidSupplier;
    private boolean done = false;
    private Timer timer = new Timer();
    private double durationSeconds = 2;
    private double yVelocity = .4;
    private ChassisSpeeds chassisSpeeds;
    
    /**
     * Constructor to make the robot aling with a branch
     * @param drivetrainsubsystem - the drivetrain subsystems
     * @param endEffector - the end effector subsystem
     * @param alignSideSupplier - a supplier of an enum that tells us if we want to align right or left
     */
    public AlignWithBranchCommand(DrivetrainSubsystem drivetrainSubsystem, EndEffectorSubsystem endEffector, Supplier<AlignToBranchSide> alignSideSupplier){
        this.drivetrain = drivetrainSubsystem;
        this.endEffector = endEffector;
        this.alignSidSupplier = alignSideSupplier;

        // explicitly not requiring the endEffector here because we are using it in a read-only capacity.
        addRequirements(drivetrainSubsystem);
    }
    

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        done = false;

        if(alignSidSupplier.get() == AlignToBranchSide.RIGHT){
            chassisSpeeds = new ChassisSpeeds(0.0, -yVelocity, 0.0);
        }
        else{
            chassisSpeeds = new ChassisSpeeds(0.0, yVelocity, 0.0);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        if(endEffector.isBranchDetected() || timer.hasElapsed(this.durationSeconds)){
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
