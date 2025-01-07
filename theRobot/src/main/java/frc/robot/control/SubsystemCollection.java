// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;
import frc.robot.subsystems.DrivetrainAccelerationSubsystem;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private CameraSubsystem cameraSubsystem = null;
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private DrivetrainAccelerationSubsystem drivetrainAccelerationSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null; 
    private LEDSubsystem ledSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    public DrivetrainSubsystem getDriveTrainSubsystem() { return driveTrainSubsystem; }
    public void setDriveTrainSubsystem(DrivetrainSubsystem value) { driveTrainSubsystem = value; }
    public boolean isDriveTrainSubsystemAvailable() { return driveTrainSubsystem != null; }

    public CameraSubsystem getCameraSubsystem() { return cameraSubsystem; }
    public void setCameraSubsystem(CameraSubsystem value) { cameraSubsystem = value; }
    public boolean isCameraSubsystemAvailable() { return cameraSubsystem != null; }

    public DrivetrainPowerSubsystem getDriveTrainPowerSubsystem() { return driveTrainPowerSubsystem; }
    public void setDriveTrainPowerSubsystem(DrivetrainPowerSubsystem value) { driveTrainPowerSubsystem = value; }
    public boolean isDriveTrainPowerSubsystemAvailable() { return driveTrainPowerSubsystem != null; }

    public DrivetrainAccelerationSubsystem getDriveTrainAccelerationSubsystem() { return drivetrainAccelerationSubsystem; }
    public void setDriveTrainAccelerationSubsystem(DrivetrainAccelerationSubsystem value) { drivetrainAccelerationSubsystem = value; }
    public boolean isDriveTrainAccelerationSubsystemAvailable() { return drivetrainAccelerationSubsystem != null; }

    public PowerDistributionPanelWatcherSubsystem getPowerDistributionPanelWatcherSubsystem() { return powerDistributionPanelWatcherSubsystem; }
    public void setPowerDistributionPanelWatcherSubsystem(PowerDistributionPanelWatcherSubsystem value) { powerDistributionPanelWatcherSubsystem = value; }
    public boolean isPowerDistributionPanelWatcherSubsystemAvailable() { return powerDistributionPanelWatcherSubsystem != null; }
    
    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }
    public boolean isManualInputInterfacesAvailable() { return manualInput != null; }

    public LEDSubsystem getLedSubsystem() { return ledSubsystem; }
    public void setLEDSubsystem(LEDSubsystem value) { ledSubsystem = value; }
    public boolean isLEDSubsystemAvailable() { return cameraSubsystem != null; }
}
