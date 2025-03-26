// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: WristSetAngleCommand.java
// Intent: Forms a command to set the coral angle
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.control.Constants;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Forms a command to get wrist in coral position
 */
public class WristSetAngleCommand extends Command {

  private WristSubsystem wristSubsystem;
  protected double desiredAngleDegrees; 

  /**
   * Constructor for WristSetAngleCommand
   * Will set wrist to desired angle before placing coral
   * @param desiredAngleDegrees
   * @param wristSubsystem
   */
  public WristSetAngleCommand(double desiredAngleDegrees, WristSubsystem wristSubsystem) {
    this.desiredAngleDegrees = desiredAngleDegrees;
    this.wristSubsystem = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setAngleDegrees(desiredAngleDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristSubsystem.isAngleWithinTolerance(desiredAngleDegrees);
  }

}