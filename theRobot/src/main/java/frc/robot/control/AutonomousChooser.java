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

import frc.robot.commands.*;

/**
 * a class for choosing different auto modes from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();

    private Command testAuto;

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

            autonomousPathChooser.setDefaultOption("Test Auto", AutonomousPath.TESTAUTO);
            SmartDashboard.putData(autonomousPathChooser);

            this.testAuto = getTestAuto();

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
            case TESTAUTO:
                return this.testAuto;
        }
        return new InstantCommand();
    }

    /**
     * A method to return the chosen auto command
     * 
     * @return command
     */
    public Command getCommand() {
        return new ParallelCommandGroup(
            getAutoPath());
    }

    private Command getTestAuto() {
        return AutoBuilder.buildAuto("TestAuto");
    }

    private enum AutonomousPath {
        TESTAUTO;
    }

    /**
     * configures the PIDs and stuff to be used for autonomous driving
     * 
     * @param subsystems
     */
    public static void configureAutoBuilder(SubsystemCollection subsystems) {

        AutoBuilder.configure(
                subsystems.getDriveTrainSubsystem()::getRobotPosition, // Pose supplier
                subsystems.getDriveTrainSubsystem()::setRobotPosition, // Position setter
                subsystems.getDriveTrainSubsystem()::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> subsystems.getDriveTrainSubsystem().driveRobotCentric(speeds), // Method that will drive
                                                                                             // the robot given ROBOT
                                                                                             // RELATIVE
                // ChassisSpeeds
                Constants.pathFollower,
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
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
         
        return false;
    }
}