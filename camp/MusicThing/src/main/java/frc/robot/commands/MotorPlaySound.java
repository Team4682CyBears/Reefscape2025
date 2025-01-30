// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MotorPlaySound extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorSubsystem m_MotorSubsystem;

  Orchestra m_Orchestra = new Orchestra();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MotorPlaySound(MotorSubsystem subsystem, MotorSubsystem subsystem2) {
    m_MotorSubsystem = subsystem;

    this.m_Orchestra.addInstrument(subsystem.m_motor, 0);
    this.m_Orchestra.addInstrument(subsystem2.m_motor, 1);
    String path = Filesystem.getDeployDirectory().getAbsolutePath() + "/track1.chrp";
    System.out.println(path);
    StatusCode status = m_Orchestra.loadMusic(path);
    
    if (!status.isOK()) {
        System.out.println("Error: " + status.toString());
    }

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    this.m_Orchestra.play();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    this.m_Orchestra.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
