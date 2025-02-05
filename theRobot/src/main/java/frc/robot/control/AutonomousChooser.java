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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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

    /**
     * Constructor for AutonomousChooser
     * 
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems) {
        this.subsystems = subsystems;

        // TODO add checks for all subsystems the autos rely on besides the drivetrain
        // here
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {

            autonomousPathChooser.setDefaultOption("Two Note", AutonomousPath.TWONOTE);
            autonomousPathChooser.addOption("Do Nothing", AutonomousPath.DONOTHING);
            SmartDashboard.putData(autonomousPathChooser);

            this.twoNote = getTwoNote();
            this.doNothing = getDoNothing();

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
            case DONOTHING:
                return this.doNothing;
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
        if (Constants.useFusedVisionInAuto) {
            fusedVisionCommand = new UseFusedVisionInAutoCommand(subsystems.getDriveTrainSubsystem());
        }

        return new ParallelCommandGroup(
                fusedVisionCommand,
                getAutoPath());
    }

    private Command getTwoNote() {
        return AutoBuilder.buildAuto("TwoNote");
    }

    private Command getDoNothing() {
        return AutoBuilder.buildAuto("DoNothing");
    }

    private enum AutonomousPath {
        TWONOTE,
        DONOTHING
    }

    /**
     * configures the PIDs and stuff to be used for autonomous driving
     * 
     * @param subsystems
     */
    public static void configureAutoBuilder(SubsystemCollection subsystems) {
        /*PPHolonomicDriveController pathFollower = new PPHolonomicDriveController(
                new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(4.5, 0.001, 0.0) // Rotation PID constants
                
        );

        AutoBuilder.configure(
                subsystems.getDriveTrainSubsystem()::getRobotPosition, // Pose supplier
                subsystems.getDriveTrainSubsystem()::setRobotPosition, // Position setter
                subsystems.getDriveTrainSubsystem()::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> subsystems.getDriveTrainSubsystem().drive(speeds), // Method that will drive
                                                                                             // the robot given ROBOT
                                                                                             // RELATIVE
                // ChassisSpeeds
                pathFollower,
                subsystems.getDriveTrainSubsystem().getPathPlannerConfig(),
                () -> false,
                subsystems.getDriveTrainSubsystem());

        // TODO add checks for all subsystems the autos rely on besides the drivetrain
        // here
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {
            NamedCommands.registerCommand("IntakeNote",
                    new ParallelCommandGroup(
                            new ButtonPressCommand("PathPlanner", "IntakeNote"),
                            new InstantCommand())); // TODO populate with real command
            NamedCommands.registerCommand("FeedNote",
                    new ParallelCommandGroup(
                            new ButtonPressCommand("PathPlanner", "FeedNote"),
                            new InstantCommand())); // TODO populate with real command
        }
        */
    }
}