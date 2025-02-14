// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: RobotContainer.java
// Intent: main robot body
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// declare package to contain current class
package frc.robot;

// import local classes
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.SolenoidSubsystem;

// import wpi libraries
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // Create instance of SubsystemCollection
  private SubsystemCollection subsystems = new SubsystemCollection();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // solenoid subsystem init
    this.initializeSolenoidSubsystem();

    // init the input system 
    this.initializeManualInputInterfaces();

    // Configure the button bindings
    if(this.subsystems.isManualInputInterfacesAvailable()) {
      DataLogManager.log(">>>> Initializing button bindings.");
      this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
      DataLogManager.log(">>>> Finished initializing button bindings.");
    }
    
    // Generally has a lot of smart dashboard commands
  }

  /**
   * A method to init the solenoid subsystem
   */
  private void initializeSolenoidSubsystem(){
    if(InstalledHardware.solenoidInstalled){
      subsystems.setSolenoidSubsystem(new SolenoidSubsystem());

      // default command for solenoid is to stop
      subsystems.getSolenoidSubsystem().setDefaultCommand(
        new InstantCommand(
          subsystems.getSolenoidSubsystem()::setAllStop, 
          subsystems.getSolenoidSubsystem()));

      DataLogManager.log("SUCCESS: SolenoidSubsystem");
    } else {
      DataLogManager.log("FAIL: SolenoidSubsystem");
    }
  }


  /**
   * A method to init the input interfaces
   */
  private void initializeManualInputInterfaces() {
    // note: in this case it is safe to build the interfaces if only one of the controllers is present
    // because button binding assignment code checks that each is installed later (see: initializeButtonCommandBindings)
    if(InstalledHardware.driverXboxControllerInstalled) {
      subsystems.setManualInputInterfaces(new ManualInputInterfaces(subsystems));
      DataLogManager.log("SUCCESS: initializeManualInputInterfaces");
    }
    else {
      DataLogManager.log("FAIL: initializeManualInputInterfaces");
    }
  }

}
