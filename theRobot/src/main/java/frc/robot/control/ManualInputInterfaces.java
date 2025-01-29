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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
  }

  /**
   * A method to return the co driver controller for rumble needs
   * @return
   */
  public final XboxController getCoDriverController() {
    return coDriverControllerForRumbleOnly;
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveX(){
    return -driverController.getLeftX();
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveY(){
    return -driverController.getLeftY();
  }

  /**
   * A method to get the spin drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputSpinDriveX(){
    return driverController.getRightX();
  }

  /**
   * A method to initialize various commands to the numerous buttons.
   * Need delayed bindings as some subsystems during testing won't always be there.
   */
  public void initializeButtonCommandBindings()
  {
    // Configure the driver xbox controller bindings
    if(InstalledHardware.driverXboxControllerInstalled){
      this.bindCommandsToDriverXboxButtons();
    }

    // Configure the co-driver xbox controller bindings
    if(InstalledHardware.coDriverXboxControllerInstalled){
      this.bindCommandsToCoDriverXboxButtons();
    }
  }

  /**
   * Will attach commands to the Driver XBox buttons 
   */
  private void bindCommandsToDriverXboxButtons(){
    if(InstalledHardware.driverXboxControllerInstalled){    

      if(this.subsystemCollection.isDriveTrainSubsystemAvailable()){
        // Back button zeros the gyroscope (as in zero yaw)
        this.driverController.back().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              subsystemCollection.getDriveTrainSubsystem()::zeroGyroscope),
            new ButtonPressCommand(
              "driverController.back()",
              "zero gyroscope")
            )
          );
        DataLogManager.log("FINISHED registering back button to zero navx ... ");
      }

      // x button press will stop all      
      this.driverController.x().onTrue(
        new ParallelCommandGroup(
          new AllStopCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "driverController.x()",
            "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")
          )
      );

      if(this.subsystemCollection.isDriveTrainPowerSubsystemAvailable() && 
         this.subsystemCollection.isDriveTrainSubsystemAvailable()){
        // left bumper press will put drivetrain in X stance
        this.driverController.leftBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setImmovableStance()
            ),
            new ButtonPressCommand(
            "driverController.leftBumper()",
            "x-stance / immovable")
          )
        );
        // left bumper release will put drivetrain in normal drive mode  
        this.driverController.leftBumper().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().unsetImmovableStance()
            ),
            new ButtonPressCommand(
            "driverController.leftBumper().onFalse()",
            "back to normal driving")
          )
        );

        // right bumper press will put drivetrain in X stance
        this.driverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setImmovableStance()
            ),
            new ButtonPressCommand(
            "driverController.rightBumper()",
            "x-stance / immovable")
          )
        );
        // right bumper release will put drivetrain in normal drive mode  
        this.driverController.rightBumper().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().unsetImmovableStance()
            ),
            new ButtonPressCommand(
            "driverController.rightBumper().onFalse()",
            "back to normal driving")
          )
        );

        // left trigger press will ramp down drivetrain to reduced speed mode 
        this.driverController.leftTrigger().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::setReducedPowerReductionFactor,
            subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
            "driverController.leftTrigger()",
            "ramp down to reduced speed")
          )
        );
        // left trigger de-press will ramp up drivetrain to max speed
        this.driverController.leftTrigger().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::resetPowerReductionFactor,
            subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
            "driverController.leftTrigger()",
            "ramp up to default speed")
          )
        );
        }      
    }
  }
  
  /**
   * Will attach commands to the Co Driver XBox buttons 
   */
  private void bindCommandsToCoDriverXboxButtons()
  {
    if(InstalledHardware.coDriverXboxControllerInstalled)
    {
      // x button press will stop all
      this.coDriverController.x().onTrue(
        new ParallelCommandGroup(
          new AllStopCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "coDriverController.x()",
            "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")
          )
        );

      if(this.subsystemCollection.isDriveTrainSubsystemAvailable()) {

      // right bumper press will toggle drivetrain reduced acceleration mode
        this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainAccelerationSubsystem().togglePowerReductionFactor()
            ),
            new ButtonPressCommand(
            "coDriverController.rightBumper()",
            "toggle limited acceleration mode")
          )
        );
      }
    }
  }
}
