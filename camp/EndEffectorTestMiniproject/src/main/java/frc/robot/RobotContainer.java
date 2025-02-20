// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: main robot body
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    SmartDashboard.putNumber("Algae Speed", 0.2);
    SmartDashboard.putNumber("Handoff Speed", 0.3);
    SmartDashboard.putNumber("Scoring Speed", 0.4);

    driverController.y().whileTrue(new ClearAlgaeCommand(endEffectorSubsystem));
    driverController.b().whileTrue(new HandoffCoralCommand(endEffectorSubsystem));
    driverController.a().whileTrue(new ScoreCoralCommand(endEffectorSubsystem));
    driverController.x().onTrue(stopEndEffectorCommand);
  }
}