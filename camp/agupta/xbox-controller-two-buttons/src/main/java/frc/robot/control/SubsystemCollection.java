// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// declare package to contain class
package frc.robot.control;

// import local class
import frc.robot.subsystems.SolenoidSubsystem;

public class SubsystemCollection {
   
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private SolenoidSubsystem solenoidSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    // getters/setters for ManualInputInterfaces
    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }
    public boolean isManualInputInterfacesAvailable() { return manualInput != null; }
 
    // solenoid subsystem
    // Uses solenoid motors, so using it for the 
    public SolenoidSubsystem getSolenoidSubsystem() { return solenoidSubsystem; }
    public void setSolenoidSubsystem(SolenoidSubsystem value) { solenoidSubsystem = value; }
    public boolean isSolenoidSubsystemAvailable() { return solenoidSubsystem != null; } // 
}
