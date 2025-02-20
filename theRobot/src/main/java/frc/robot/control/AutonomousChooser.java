// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: AutonomousChooser.java
// Intent: Allows auto mode routine to be selected from shuffleboard
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.commands.AlignWithReefCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.UseFusedVisionInAutoCommand;

/**
 * a class for choosing different auto modes from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();

    private Command twoNote;
    private Command doNothing;
    private Command TestAutoPlanner;
    private Command TestAutoCommand;

    /**
     * Constructor for AutonomousChooser
     * 
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        // TODO need to make sure that mirroing works for red/blue paths
        // specifically we want to insure that when we are on the red(mirrored) side we really do mirror

        // TODO add checks for all subsystems the autos rely on besides the drivetrain
        // here
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {

            autonomousPathChooser.setDefaultOption("Two Note", AutonomousPath.TWONOTE);
            SmartDashboard.putData(autonomousPathChooser);

            this.twoNote = getTwoNote();

        } else {
            DataLogManager.log(">>>>> NO auto routine becuase missing subsystems");
        }
    }

    /**
     * returns the path planner auto to be used in auto period
     * 
     * @return command
     */
    public Command getAutoPath() {
        switch (autonomousPathChooser.getSelected()) {
            case TWONOTE:
                return this.twoNote;
        }
        return new InstantCommand();
    }

    /**
     * A method to return the chosen auto command
     * 
     * @return command
     */
    public Command getCommand() {
        Command fusedVisionCommand = new InstantCommand();
        if (!Constants.useFusedVisionInAuto) {
            fusedVisionCommand = new UseFusedVisionInAutoCommand(subsystems.getDriveTrainSubsystem());
        }

        return new ParallelCommandGroup(
                fusedVisionCommand,
                getAutoPath());
    }

    private Command getTwoNote() {
        return AutoBuilder.buildAuto("TwoNote");
    }

    private enum AutonomousPath {
        TWONOTE;
    }

    /**
     * configures the PIDs and stuff to be used for autonomous driving
     * 
     * @param subsystems
     */
    public static void configureAutoBuilder(SubsystemCollection subsystems) {
        PPHolonomicDriveController pathFollower = new PPHolonomicDriveController(
                new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(4.5, 0.001, 0.0) // Rotation PID constants
                
        );

        AutoBuilder.configure(
                subsystems.getDriveTrainSubsystem()::getRobotPosition, // Pose supplier
                subsystems.getDriveTrainSubsystem()::setRobotPosition, // Position setter
                subsystems.getDriveTrainSubsystem()::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> subsystems.getDriveTrainSubsystem().driveRobotCentric(speeds), // Method that will drive
                                                                                             // the robot given ROBOT
                                                                                             // RELATIVE
                // ChassisSpeeds
                pathFollower,
                subsystems.getDriveTrainSubsystem().getPathPlannerConfig(),
                () -> getShouldMirrorPath(),
                subsystems.getDriveTrainSubsystem());

        // TODO add checks for all subsystems the autos rely on besides the drivetrain
        // here
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {
            NamedCommands.registerCommand("AlignWithReef", new AlignWithReefCommand(subsystems, false));
        }
        
    }

    public static boolean getShouldMirrorPath(){
        return DriverStation.getAlliance().get() == Alliance.Red;
    }
}