// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.WristSetAngleCommand;
import frc.robot.control.Constants;
import frc.robot.control.Constants.OperatorConstants;
import frc.robot.subsystems.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final WristSubsystem WristSubsystem = new WristSubsystem();

  // create instance of xbox controller
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    System.out.println("!!!!!!!!!!!CONFIGURING BUTTON BINDINGS!!!!!!!!!!!!!!");

    // Sets the default command to "stop." This way, when no buttons are being pressed, the motor doesn't move.
    //this.WristSubsystem.setDefaultCommand(); 

    // TODO: Figure out if we are using subsystem collection, if so call the getter from that function instead of direct
   
    // Rotate to shooterAngle
    driverController.a().onTrue(new WristSetAngleCommand(
      Constants.shooterAngle, 
      this.WristSubsystem));
        //new WristSetAngleCommand(Constants.shooterAngleShootFromSpeaker,this.subsystemCollection.getWristSubsystem());

    // Rotate to algaeRemoverAngle
    driverController.b().onTrue(new WristSetAngleCommand(
      Constants.algaeAngle, 
      this.WristSubsystem));

    //this.talonMotorSubsystem.setDefaultCommand(new StopCommand(this.talonMotorSubsystem)); // Sets the default command to "stop." This way, when no buttons are being pressed, the motor doesn't move.
    
    //driverController.b().whileTrue(new WindCommand(this.talonMotorSubsystem)); // Binds the "b" button to the backwards command.
    //driverController.a().whileTrue(new UnwindCommand(this.talonMotorSubsystem)); // Binds the "a" button to the forward command.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}
