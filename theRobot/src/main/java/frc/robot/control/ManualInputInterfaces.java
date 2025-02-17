// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;

public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController);
  private XboxController driverControllerForRumbleOnly = new XboxController(Constants.portDriverController);
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
  private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection) {
    subsystemCollection = currentCollection;
  }

  /**
   * A method to return the co driver controller for rumble needs
   * 
   * @return
   */
  public final XboxController getCoDriverController() {
    return coDriverControllerForRumbleOnly;
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * 
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveX() {
    return -driverController.getLeftX();
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * 
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveY() {
    return -driverController.getLeftY();
  }

  /**
   * A method to get the spin drive X componet being input from humans
   * 
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputSpinDriveX() {
    return driverController.getRightX();
  }

  /**
   * A method to initialize various commands to the numerous buttons.
   * Need delayed bindings as some subsystems during testing won't always be
   * there.
   */
  public void initializeButtonCommandBindings() {
    // Configure the driver xbox controller bindings
    if (InstalledHardware.driverXboxControllerInstalled) {
      this.bindCommandsToDriverXboxButtons();
    }

    // Configure the co-driver xbox controller bindings
    if (InstalledHardware.coDriverXboxControllerInstalled) {
      this.bindCommandsToCoDriverXboxButtons();
    }
  }

  /**
   * Will attach commands to the Driver XBox buttons
   */
  private void bindCommandsToDriverXboxButtons() {
    if (InstalledHardware.driverXboxControllerInstalled) {

      if (this.subsystemCollection.isDriveTrainSubsystemAvailable()) {
        // Back button zeros the gyroscope (as in zero yaw)
        this.driverController.back().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    subsystemCollection.getDriveTrainSubsystem()::zeroGyroscope),
                new ButtonPressCommand(
                    "driverController.back()",
                    "zero gyroscope")));
        DataLogManager.log("FINISHED registering back button to zero gyroscope ... ");
      }

      // x button press will stop all
      this.driverController.x().onTrue(
          new ParallelCommandGroup(
              new AllStopCommand(
                  this.subsystemCollection),
              new ButtonPressCommand(
                  "driverController.x()",
                  "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")));

      this.driverController.leftBumper().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "driverController.leftBumper()",
                  "Remove Algae")));

      this.driverController.leftBumper().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "driverController.leftBumper()",
                  "Remove Algae")));

      this.driverController.leftTrigger().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::setReducedPowerReductionFactor,
                  subsystemCollection.getDriveTrainPowerSubsystem()),
              new ButtonPressCommand(
                  "driverController.leftTrigger()",
                  "ramp down to reduced speed (Pit Limiter)")));

      this.driverController.leftTrigger().onFalse(
          new ParallelCommandGroup(
              new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::resetPowerReductionFactor,
                  subsystemCollection.getDriveTrainPowerSubsystem()),
              new ButtonPressCommand(
                  "driverController.leftTrigger()",
                  "ramp up to default speed (Pit Limiter)")));

      this.driverController.a().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "driverController.a()",
                  "Align to branch")));
      this.driverController.b().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "driverController.b()",
                  "Align to reef")));
      this.driverController.y().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "driverController.y()",
                  "Shoot")));

    }
  }

  /**
   * Will attach commands to the Co Driver XBox buttons
   */
  private void bindCommandsToCoDriverXboxButtons() {
    if (InstalledHardware.coDriverXboxControllerInstalled) {
      // x button press will stop all
      this.coDriverController.x().onTrue(
          new ParallelCommandGroup(
              new AllStopCommand(
                  this.subsystemCollection),
              new ButtonPressCommand(
                  "coDriverController.x()",
                  "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")));
      this.coDriverController.y().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.y()",
                  "Move to position")));
      this.coDriverController.b().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.b()",
                  "Collapse Funnel")));
      this.coDriverController.a().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.a()",
                  "Stow elevator")));
      this.coDriverController.povLeft().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.povLeft()",
                  "L1")));
      this.coDriverController.povDown().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.povDown()",
                  "L2")));
      this.coDriverController.povRight().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.povRight()",
                  "L3")));
      this.coDriverController.povUp().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.povUp()",
                  "L4")));
      this.coDriverController.leftBumper().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.leftBumper()",
                  "Collapse Funnel")));
      this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.rightBumper()",
                  "Reel Climber")));
      this.coDriverController.leftTrigger().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.leftTrigger()",
                  "Align Left")));
      this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
              new InstantCommand(), // TODO: Fill with real command
              new ButtonPressCommand(
                  "coDriverController.rightBumper()",
                  "Align Right")));
    }
  }
}
