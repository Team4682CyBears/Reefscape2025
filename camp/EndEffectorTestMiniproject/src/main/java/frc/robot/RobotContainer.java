// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClearAlgaeCommand;
import frc.robot.commands.HandoffCoralCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.StopEndEffectorCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class RobotContainer {

  private final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(Constants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  // Configure inputs for the EndEffector
  private void configureBindings() {
    StopEndEffectorCommand stopEndEffectorCommand = new StopEndEffectorCommand(endEffectorSubsystem);
    endEffectorSubsystem.setDefaultCommand(stopEndEffectorCommand);

    driverController.y().whileTrue(new ClearAlgaeCommand(endEffectorSubsystem));
    driverController.b().onTrue(new HandoffCoralCommand(endEffectorSubsystem));
    driverController.a().whileTrue(new ScoreCoralCommand(endEffectorSubsystem));
    driverController.x().onTrue(stopEndEffectorCommand);
  }
}