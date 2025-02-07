package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.RobotPosesForReef;
import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class AlignWithReefCommand extends Command {
    private Timer timer = new Timer();
    private double timeout = 1.0;
    private boolean done = false;
    private DrivetrainSubsystem drivetrain;
    private CameraSubsystem camera;

    private boolean foundReefTag = false;

    private double tagID;

    // TODO These velocities and acccelerations were copied from Ted. May need to be changed for new robot. 
    private double maxVelocityMPS = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    private double maxAccelerationPMSSq = 6; // 6.0 max
    private double maxAngularVelocityRadPerSecond = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    private double maxAngularAccelerationRadPerSecondSq = 12.0; // 12.0 max

    private PPHolonomicDriveController pathFollower = new PPHolonomicDriveController(
        new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(4.5, 0.001, 0.0) // Rotation PID constants
    );



    public AlignWithReefCommand(DrivetrainSubsystem drivetrainSubsystem, CameraSubsystem cameraSubsystem){
        this.drivetrain = drivetrainSubsystem;

        this.camera = cameraSubsystem;
    }

    public AlignWithReefCommand(DrivetrainSubsystem drivetrainSubsystem, CameraSubsystem cameraSubsystem, double timoutSeconds){
        this.drivetrain = drivetrainSubsystem;

        this.camera = cameraSubsystem;

        this.timeout = timoutSeconds;
    }

    @Override
    public void initialize(){
        foundReefTag = false;
        tagID = -1;
        timer.reset();
        timer.start();
        done = false;       
    }

    @Override
    public void execute(){
        if(timer.hasElapsed(this.timeout)){
            done = true;
        }
        if(drivetrain.getRobotPosition().getTranslation().getDistance(RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getTranslation()) <= .1){
            done = true;
        }
        else if (foundReefTag) {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(drivetrain.getRobotPosition().getTranslation(), new Rotation2d(0)), 
                                                                        new Pose2d(RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getTranslation(), new Rotation2d(0)));

            PathPlannerPath traj = new PathPlannerPath(
                waypoints,
                getPathConstraints(),
                null,
                new GoalEndState(0.0, 
                RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getRotation())
            );

            new FollowPathCommand(
                traj,
                drivetrain::getRobotPosition, // Pose supplier
                drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drivetrain.driveRobotCentric(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
                // We do not currently use the module feedforwards
                pathFollower,
                drivetrain.getPathPlannerConfig(), // The robot configuration
                () -> mirrorPathForRedAliance(),
                (Subsystem) drivetrain).schedule();
            done = true;
        }
        else{
            tagID = camera.getTagId();
            
            if ((tagID <= 11 && tagID >= 6) || (tagID <= 22 && tagID >= 17)){
                foundReefTag = true;
            }
        }  
    }

    @Override
    public boolean isFinished(){
        return done;
    }

    private PathConstraints getPathConstraints(){
        return new PathConstraints(
          maxVelocityMPS, 
          maxAccelerationPMSSq, 
          maxAngularVelocityRadPerSecond, 
          maxAngularAccelerationRadPerSecondSq);
    }

    private boolean mirrorPathForRedAliance(){
        //when using paths generated from april tag coords always turn mirroring off
        return false;
    }

}