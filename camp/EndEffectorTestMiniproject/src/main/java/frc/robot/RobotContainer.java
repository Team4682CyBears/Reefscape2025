// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.commands.MotorDefaultCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final MotorSubsystem motorSubsystem = new MotorSubsystem(Constants.motorPort);

  public RobotContainer() {
    SmartDashboard.putNumber("Motor Speed", 0.0);
    SmartDashboard.putData("Set Motor Speed", new MotorDefaultCommand(
      motorSubsystem,
      () -> SmartDashboard.getNumber("Motor Speed", 0.0)
    ));
  }
}
