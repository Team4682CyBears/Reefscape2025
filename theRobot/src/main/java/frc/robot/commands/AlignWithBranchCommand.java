package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlignWithBranchCommand extends Command{
    private DrivetrainSubsystem drivetrain;
    private EndEffectorSubsystem endEffector;
    private boolean aligningRight;
    private boolean done = false;
    private Timer timer = new Timer();
    private double durationSeconds = 1;
    private double yVelocity = .6;
    private ChassisSpeeds chassisSpeeds;
    

    public AlignWithBranchCommand(DrivetrainSubsystem drivetrainSubsystem, EndEffectorSubsystem endEffector, boolean aligningRight){
        this.drivetrain = drivetrainSubsystem;
        this.endEffector = endEffector;
        this.aligningRight = aligningRight;

        // explicitly not requiring the endEffector here because we are using it in a read-only capacity.
        addRequirements(drivetrainSubsystem);

    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        done = false;

        if(aligningRight){
            chassisSpeeds = new ChassisSpeeds(0.0, -yVelocity, 0.0);
        }
        else{
            chassisSpeeds = new ChassisSpeeds(0.0, yVelocity, 0.0);
        }
    }

    @Override
    public void execute(){
        if(endEffector.isBranchDetected() || timer.hasElapsed(this.durationSeconds)){
            done = true;
        }
        else {
            drivetrain.driveRobotCentric(chassisSpeeds);
        }
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.driveFieldCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
        if(interrupted)
        {
        done = true;      
        }
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}
