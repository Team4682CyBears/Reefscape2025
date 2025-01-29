// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MotorPlaySound extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorSubsystem m_MotorSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MotorPlaySound(MotorSubsystem subsystem) {
    m_MotorSubsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_MotorSubsystem.playMusic();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_MotorSubsystem.stopMusic();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
