// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: AlignWithReefCommand.java
// Intent: Forms a to align with an april tag on a reef
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

/**
 *  Class to form a command that will align with an april tag on a reef
 * */
public class AlignWithReefCommand extends Command {
    private Timer timer = new Timer();
    private double timeoutSeconds = 2;
    private boolean done = false;
    private DrivetrainSubsystem drivetrain;
    private CameraSubsystem camera;

    private boolean foundReefTag = false;

    private boolean alreadyInitializedCommand = false;

    private double tagID;

    FollowPathCommand followPathCommand;

    // TODO These velocities and acccelerations were copied from Ted. May need to be
    // changed for new robot.
    // private double maxVelocityMPS =
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
    private double maxVelocityMPS = 5.3;
    private double maxAccelerationPMSSq = 4; // 6.0 max
    private double maxAngularVelocityRadPerSecond = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    private double maxAngularAccelerationRadPerSecondSq = 10.0; // 12.0 max

    private PPHolonomicDriveController pathFollower = new PPHolonomicDriveController(
            new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(4.5, 0.001, 0.0) // Rotation PID constants
    );

    /**
     * Constructor to make a command to align with an april tag on the reef
     * @param drivetrainSubsystem - the drivetrain subsystem
     * @param cameraSubsystem - the camera subsystem
     */
    public AlignWithReefCommand(DrivetrainSubsystem drivetrainSubsystem, CameraSubsystem cameraSubsystem) {
        this.drivetrain = drivetrainSubsystem;

        this.camera = cameraSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("Starting AlignWithReefCommand.!!!!!!!!!!!!!!!");
        foundReefTag = false;
        alreadyInitializedCommand = false;
        tagID = -1;
        timer.reset();
        timer.start();
        done = false;
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(this.timeoutSeconds)) {
            System.out.println("AlignWithReefCommand Timer has expired. Setting done = true");
            done = true;
        } else if (foundReefTag && !alreadyInitializedCommand) {
            System.out.println("AlignWithReefCommand found reef tag and calculating trajectory");
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    new Pose2d(drivetrain.getRobotPosition().getTranslation(), new Rotation2d(0)),
                    new Pose2d(RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getTranslation(),
                            new Rotation2d(0)));

            System.out.println("Path Waypoints: " + waypoints.toString());

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    getPathConstraints(),
                    null,
                    new GoalEndState(0.0,
                            RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getRotation()));

            PathPlannerTrajectory traj = new PathPlannerTrajectory(path, drivetrain.getChassisSpeeds(),
                    drivetrain.getGyroscopeRotation(), drivetrain.getPathPlannerConfig());

            if (!Double.isNaN(traj.getTotalTimeSeconds())) {
                System.out.println("Trajectory total time: " + traj.getTotalTimeSeconds());
                followPathCommand = new FollowPathCommand(
                        path,
                        drivetrain::getRobotPosition, // Pose supplier
                        drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        (speeds, feedforwards) -> drivetrain.driveRobotCentric(speeds), // Method that will drive
                                                                                        // the robot given ROBOT
                                                                                        // RELATIVE ChassisSpeeds.
                        // We do not currently use the module feedforwards
                        pathFollower,
                        drivetrain.getPathPlannerConfig(), // The robot configuration
                        () -> mirrorPathForRedAliance(),
                        (Subsystem) drivetrain);

                drivetrain.setUseVision(false);

                System.out.println("Scheduling follow Path command!!");
                followPathCommand.andThen(() -> System.out.println("ENDING FOL LOW PATH COMMAND"))
                        .andThen(() -> drivetrain.setUseVision(true));

                followPathCommand.initialize();

                alreadyInitializedCommand = true;

                timer.reset();

                timeoutSeconds = traj.getTotalTimeSeconds();
            } else {
                System.out.println("PATH HAD NAN TIME !!!!!! ");
            }
        } else if (!alreadyInitializedCommand){
            tagID = camera.getTagId();

            if ((tagID <= 11 && tagID >= 6) || (tagID <= 22 && tagID >= 17)) {
                foundReefTag = true;
            }
        } else {
            followPathCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if(alreadyInitializedCommand){
            return followPathCommand.isFinished();
        }
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        if(alreadyInitializedCommand){
            followPathCommand.end(interrupted);
        }
        System.out.println("Ending AlignWithReefCommand!!");
        if (interrupted) {
            System.out.println("AlignWithReefCommand was interrupted.");
            done = true;
        }
        drivetrain.setUseVision(true);
        System.out.println("Comppled closout of AlignWithReefCommand!!");
    }

    private PathConstraints getPathConstraints() {
        return new PathConstraints(
                maxVelocityMPS,
                maxAccelerationPMSSq,
                maxAngularVelocityRadPerSecond,
                maxAngularAccelerationRadPerSecondSq);
    }

    private boolean mirrorPathForRedAliance() {
        // when using paths generated from april tag coords always turn mirroring off
        return false;
    }

}