// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.commands.MotorDefaultCommand;

public class RobotContainer {

  private final MotorSubsystem motorSubsystem = new MotorSubsystem(Constants.motorPort);

  public RobotContainer() {
    SmartDashboard.putNumber("Motor Speed", 0.0);
    SmartDashboard.putData("Set Motor Speed", new MotorDefaultCommand(
      motorSubsystem,
      // Clamped between [-1, 1] in MotorSubsystem
      () -> SmartDashboard.getNumber("Motor Speed", 0.0)
    ));
  }
}