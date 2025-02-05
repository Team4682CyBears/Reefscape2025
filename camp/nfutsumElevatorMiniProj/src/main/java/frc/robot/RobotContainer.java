// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveToPositionCommand;
import frc.robot.common.ElevatorPositions;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveToPositionCommand;
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
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
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
  public RobotContainer() {
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
  private void configureBindings() {
    // dpad up. while true. new RunCommand(elevatorSubsystem::moveUp, elevatorSubsystem);
    // dpad down. ""
    // create enum for elevator positions (stow=0, L2, L3, L4)
    // create a moveToPosition command (takes in enum value and moves to a specific distance)
    // init calls the elevator move to position method
    // isFinished checks the isAtTargetHeight and completes once it's at height or a timeout. 
    // mapping betweeen postiion and distance is in Constants. (make them type Distance)
    // Y -> moveToPosition(L4);
    // B -> moveToPosition(L3);
    // A -> moveToPosition(L2);
    // X -> moveToPosition(STOW);

    elevatorSubsystem.setDefaultCommand(new RunCommand(elevatorSubsystem::stopElevator, elevatorSubsystem));

    driverController.povUp().whileTrue(new RunCommand(elevatorSubsystem::moveUp, elevatorSubsystem));
    driverController.povDown().whileTrue(new RunCommand(elevatorSubsystem::moveDown, elevatorSubsystem));

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