// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: holds robot subsystems and commands, and is where most of the declarative robot setup (e.g. button bindings) is performed
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// defined namespace for this class
package frc.robot;

// import local classes
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveToPositionCommand;
import frc.robot.common.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;

// import wpilib libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // create an instance of elevatorsubsystem
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  // create instance of move to each 4 elevator position
  private final MoveToPositionCommand positionL4 = new MoveToPositionCommand(elevatorSubsystem, ElevatorPositions.L4);
  private final MoveToPositionCommand positionL3 = new MoveToPositionCommand(elevatorSubsystem, ElevatorPositions.L3);
  private final MoveToPositionCommand positionL2 = new MoveToPositionCommand(elevatorSubsystem, ElevatorPositions.L2);
  private final MoveToPositionCommand positionStow = new MoveToPositionCommand(elevatorSubsystem, ElevatorPositions.STOW);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { // constructor
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  // configure button bindings
  private void configureBindings() {

    // set default elevator to off
    elevatorSubsystem.setDefaultCommand(
      new RunCommand(
        elevatorSubsystem::stopElevator, 
        elevatorSubsystem));

    // move motor forward/up with up toggle
    driverController.povUp().whileTrue(
      new RunCommand(
        elevatorSubsystem::moveUp, 
        elevatorSubsystem));

    // move motor backward/down with down toggle
    driverController.povDown().whileTrue(
      new RunCommand(
        elevatorSubsystem::moveDown, 
        elevatorSubsystem));
    
    // configure face buttons to elevator presets
    driverController.y().onTrue(positionL4);
    driverController.b().onTrue(positionL3);
    driverController.a().onTrue(positionL2);
    driverController.x().onTrue(positionStow);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}