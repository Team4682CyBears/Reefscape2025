package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.RobotPosesForReef;
import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignWithReefCommand extends Command {
    private Timer timer = new Timer();
    private double timeout = 3.0;
    private boolean done = false;
    private DrivetrainSubsystem drivetrain;
    private CameraSubsystem camera;

    private boolean foundReefTag = false;

    private double tagID;
    private Pose2d destination;

    public AlignWithReefCommand(DrivetrainSubsystem drivetrainSubsystem, CameraSubsystem cameraSubsystem){
        this.drivetrain = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);

        this.camera = cameraSubsystem;
    }

    public AlignWithReefCommand(DrivetrainSubsystem drivetrainSubsystem, CameraSubsystem cameraSubsystem, double timoutSeconds){
        this.drivetrain = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);

        this.camera = cameraSubsystem;

        this.timeout = timoutSeconds;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        done = false;       
    }

    @Override
    public void execute(){
        if(timer.hasElapsed(this.timeout)){
            done = true;
        }
    }

    @Override
    public boolean isFinished(){
        return done;
    }

}