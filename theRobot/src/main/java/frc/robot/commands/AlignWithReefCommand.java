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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.RobotPosesForReef;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.Constants;
import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

/**
 * Class to form a command that will make a command to align with an april tag on a reef
 */
public class AlignWithReefCommand extends Command {
    private Timer timer = new Timer();
    private double timeoutSeconds = 2;
    private boolean done = false;
    private DrivetrainSubsystem drivetrain;
    private CameraSubsystem camera;
    private SubsystemCollection subsystemCollection;

    private double tagID;

    private boolean shouldAlignBranch;

    private PathPlannerPath path;

    private enum Stage {
        LOOKINGFORTAG,
        GETTINGVALIDPATH,
        DRIVINGCOMMAND
    }

    private Stage stage = Stage.LOOKINGFORTAG;

    FollowPathCommand followPathCommand;

    private double maxVelocityMPS = 5.3;
    private double maxAccelerationPMSSq = 3.5; // 6.0 max
    private double maxAngularVelocityRadPerSecond = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    private double maxAngularAccelerationRadPerSecondSq = 10.0; // 12.0 max

    /**
     * Constructor to make a command to align with an april tag on the reef
     * 
     * @param drivetrainSubsystem - the drivetrain subsystem
     * @param cameraSubsystem     - the camera subsystem
     */
    public AlignWithReefCommand(SubsystemCollection subsystemCollection, boolean shouldAlignBranch) {
        this.subsystemCollection = subsystemCollection;
        this.drivetrain = this.subsystemCollection.getDriveTrainSubsystem();
        this.camera = this.subsystemCollection.getCameraSubsystem();

        this.shouldAlignBranch = shouldAlignBranch;

        //We dont require any subsystems because we call a command later that requires the drivetrain
    }

    @Override
    public void initialize() {
        stage = Stage.LOOKINGFORTAG;
        tagID = -1;
        timer.reset();
        timer.start();
        done = false;
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(this.timeoutSeconds)) {
            done = true;
        }
        switch (stage) {
            case LOOKINGFORTAG:
                tagID = camera.getTagId();

                if (RobotPosesForReef.isReefTag(tagID)) {
                    stage = Stage.GETTINGVALIDPATH;
                }
                break;
            case GETTINGVALIDPATH:
                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                        new Pose2d(drivetrain.getRobotPosition().getTranslation(), new Rotation2d(0)),
                        new Pose2d(RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getTranslation(),
                                new Rotation2d(0)));

                path = new PathPlannerPath(
                        waypoints,
                        Constants.autoAlignPathConstraints,
                        null,
                        new GoalEndState(0.0, RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getRotation()));

                PathPlannerTrajectory traj = new PathPlannerTrajectory(path, drivetrain.getChassisSpeeds(),
                        drivetrain.getGyroscopeRotation(), drivetrain.getPathPlannerConfig());

                //We cant run a path shorter than .5meters this is "intended" by path planner
                if (!Double.isNaN(traj.getTotalTimeSeconds()) && drivetrain.getRobotPosition().getTranslation().getDistance(RobotPosesForReef.getPoseFromTagIDWithOffset(tagID).getTranslation()) >= 0.5){
                    followPathCommand = new FollowPathCommand(
                            path,
                            drivetrain::getRobotPosition, // Pose supplier
                            drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                            (speeds, feedforwards) -> drivetrain.driveRobotCentric(speeds), // Method that will drive
                                                                                            // the robot given ROBOT
                                                                                            // RELATIVE ChassisSpeeds.
                            // We do not currently use the module feedforwards
                            Constants.pathFollower,
                            drivetrain.getPathPlannerConfig(), // The robot configuration
                            () -> mirrorPathForRedAliance(),
                            (Subsystem) drivetrain);

                    stage = Stage.DRIVINGCOMMAND;
                }
                break;
            case DRIVINGCOMMAND:
                drivetrain.setUseVision(false);
        
                //We are launching a string of commands from this command because we want to use path planners follow path command and then do stuff after
                followPathCommand.andThen(() -> drivetrain.setUseVision(true))
                        .andThen(new ConditionalCommand(
                                new AlignToBranchCommand(drivetrain,
                                        this.subsystemCollection.getBranchDetectorSubsystem(),
                                        () -> this.subsystemCollection.getAlignWithBranchDirection().getAlignWithBranchSide()),
                                new InstantCommand(),
                                () -> shouldAlignBranch))
                        .schedule();
                

                done = true;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            done = true;
        }
        drivetrain.setUseVision(true);
    }

    /**
     * a method to return wether or not we want to mirror the path
     * @return when using paths generated from april tag coords always turn mirroring off
     */
    private boolean mirrorPathForRedAliance() {
        return false;
    }
}