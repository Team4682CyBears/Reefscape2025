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
import frc.robot.common.AlignToBranchSide;
import frc.robot.common.ElevatorPositions;

/**
 * a class for choosing different auto modes from shuffleboard
 */
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();

    private Command testAuto;
    private Command DoNothing;
    private Command Mobility;
    private Command L0;
    private Command L1;
    private Command L2;
    private Command L3;
    private Command L4;

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

            autonomousPathChooser.setDefaultOption("Do Nothing", AutonomousPath.DONOTHING);
            autonomousPathChooser.addOption("Test Auto", AutonomousPath.TESTAUTO);
            autonomousPathChooser.addOption("Mobility", AutonomousPath.MOBILITY);
            autonomousPathChooser.addOption("L0", AutonomousPath.L0AUTO);
            autonomousPathChooser.addOption("L1", AutonomousPath.L1AUTO);
            autonomousPathChooser.addOption("L2", AutonomousPath.L2AUTO);
            autonomousPathChooser.addOption("L3", AutonomousPath.L3AUTO);
            autonomousPathChooser.addOption("L4", AutonomousPath.L4AUTO);
            SmartDashboard.putData(autonomousPathChooser);

            this.testAuto = getTestAuto();
            this.DoNothing = getDoNothing();
            this.Mobility = getMobilityAuto();
            this.L0 = getL0Auto();
            this.L1 = getL1Auto();
            this.L2 = getL2Auto();
            this.L3 = getL3Auto();
            this.L4 = getL4Auto();

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
            case DONOTHING:
                return this.DoNothing;
            case MOBILITY:
                return this.Mobility;
            case L0AUTO:
                return this.L0;
            case L1AUTO:
                return this.L1;
            case L2AUTO:
                return this.L2;
            case L3AUTO:
                return this.L3;
            case L4AUTO:
                return this.L4;
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

    private Command getDoNothing() {
        return new InstantCommand();
    }

    private Command getMobilityAuto() {
        return AutoBuilder.buildAuto("Mobility");
    }

    private Command getL0Auto() {
        return AutoBuilder.buildAuto("L0");
    }

    private Command getL1Auto(){
        return AutoBuilder.buildAuto("TopL1");
    }

    private Command getL2Auto(){
        return AutoBuilder.buildAuto("BottomL1");
    }

    private Command getL3Auto(){
        return AutoBuilder.buildAuto("L3");
    }

    private Command getL4Auto(){
        return AutoBuilder.buildAuto("L4");
    }

    private enum AutonomousPath {
        TESTAUTO,
        DONOTHING,
        MOBILITY,
        L0AUTO,
        L1AUTO,
        L2AUTO,
        L3AUTO,
        L4AUTO
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

        // Register named commands
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {
            NamedCommands.registerCommand("AlignWithReef", new AlignWithReefCommand(subsystems, false));
            NamedCommands.registerCommand("Align Branch Right", new AlignToBranchCommand(subsystems.getDriveTrainSubsystem(), subsystems.getBranchDetectorSubsystem(), () -> AlignToBranchSide.RIGHT));
            NamedCommands.registerCommand("Align Branch Left", new AlignToBranchCommand(subsystems.getDriveTrainSubsystem(), subsystems.getBranchDetectorSubsystem(), () -> AlignToBranchSide.LEFT));
        }
        if(subsystems.isElevatorSubsystemAvailable()) {
            NamedCommands.registerCommand("Reset Elevator Position", new MoveToPositionCommand(subsystems.getElevatorSubsystem(), () -> ElevatorPositions.STOW));
            NamedCommands.registerCommand("L1", new MoveToPositionCommand(subsystems.getElevatorSubsystem(), () -> ElevatorPositions.L1));
            NamedCommands.registerCommand("L2", new MoveToPositionCommand(subsystems.getElevatorSubsystem(), () -> ElevatorPositions.L2));
            NamedCommands.registerCommand("L3", new MoveToPositionCommand(subsystems.getElevatorSubsystem(), () -> ElevatorPositions.L3));
            NamedCommands.registerCommand("L4", new MoveToPositionCommand(subsystems.getElevatorSubsystem(), () -> ElevatorPositions.L4));
            NamedCommands.registerCommand("Sense", new MoveToPositionCommand(subsystems.getElevatorSubsystem(), () -> ElevatorPositions.SENSE));
        }
        if(subsystems.isEndEffectorSubsystemAvailable()) {
            NamedCommands.registerCommand("Score Piece", new ScoreCoralCommand(subsystems.getEndEffectorSubsystem()).withTimeout(.3));
            NamedCommands.registerCommand("Intake Piece", new IntakeCoralCommand(subsystems.getEndEffectorSubsystem()));
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