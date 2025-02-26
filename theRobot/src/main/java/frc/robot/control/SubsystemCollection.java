// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.BranchDetectorSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class SubsystemCollection {
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems
    private CameraSubsystem cameraSubsystem = null;
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private BranchDetectorSubsystem branchDetectorSubsystem = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null;
    private LEDSubsystem ledSubsystem = null;
    private AlignWithBranchDirection alignWithBranchDirection = null;
    private Object elevatorSubsystem = null; // TODO: Replace with real subsystem
    private Object endEffectorSubsystem = null; // TODO: Replace with real subsystem
    private Object funnelSubsystem = null; // TODO: Replace with real subsystem
    private WristSubsystem shooterAngleSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {
    }

    public DrivetrainSubsystem getDriveTrainSubsystem() {
        return driveTrainSubsystem;
    }
    public boolean isDriveTrainSubsystemAvailable() {
        return driveTrainSubsystem != null;
    }
    public void setDriveTrainSubsystem(DrivetrainSubsystem value) {
        driveTrainSubsystem = value;
    }

    public BranchDetectorSubsystem getBranchDetectorSubsystem() { return branchDetectorSubsystem; }
    public void setBranchDetectorSubsystem(BranchDetectorSubsystem value) { branchDetectorSubsystem = value; }
    public boolean isBranchDetectorSubsystemAvailable() { return branchDetectorSubsystem != null; }

    public CameraSubsystem getCameraSubsystem() { return cameraSubsystem; }
    public void setCameraSubsystem(CameraSubsystem value) { cameraSubsystem = value; }
    public boolean isCameraSubsystemAvailable() { return cameraSubsystem != null; }

    public LEDSubsystem getLedSubsystem() { return ledSubsystem; }
    public void setLEDSubsystem(LEDSubsystem value) { ledSubsystem = value; }
    public boolean isLEDSubsystemAvailable() { return cameraSubsystem != null; }

    public AlignWithBranchDirection getAlignWithBranchDirection() { return alignWithBranchDirection; }
    public void setAlignWithBranchDirection(AlignWithBranchDirection value) { alignWithBranchDirection = value; }
    public boolean isAlignWithBranchDirection() { return alignWithBranchDirection != null; }

    public WristSubsystem getShooterAngleSubsystem() { return shooterAngleSubsystem; }
    public void setShooterAngleSubsystem(WristSubsystem value) { shooterAngleSubsystem = value; }
    public boolean isShooterAngleSubsystemAvailable() { return shooterAngleSubsystem != null; }

    public DrivetrainPowerSubsystem getDriveTrainPowerSubsystem() {
        return driveTrainPowerSubsystem;
    }

    public void setDriveTrainPowerSubsystem(DrivetrainPowerSubsystem value) {
        driveTrainPowerSubsystem = value;
    }

    public boolean isDriveTrainPowerSubsystemAvailable() {
        return driveTrainPowerSubsystem != null;
    }

    public PowerDistributionPanelWatcherSubsystem getPowerDistributionPanelWatcherSubsystem() {
        return powerDistributionPanelWatcherSubsystem;
    }

    public void setPowerDistributionPanelWatcherSubsystem(PowerDistributionPanelWatcherSubsystem value) {
        powerDistributionPanelWatcherSubsystem = value;
    }

    public boolean isPowerDistributionPanelWatcherSubsystemAvailable() {
        return powerDistributionPanelWatcherSubsystem != null;
    }

    public ManualInputInterfaces getManualInputInterfaces() {
        return manualInput;
    }

    public void setManualInputInterfaces(ManualInputInterfaces value) {
        manualInput = value;
    }

    public boolean isManualInputInterfacesAvailable() {
        return manualInput != null;
    }

    public Object getElevatorSubsystem() {
        return elevatorSubsystem;
    }

    public void setElevatorSubsystem(Object value) {
        elevatorSubsystem = value;
    }

    public boolean isElevatorSubsystemAvailable() {
        return elevatorSubsystem != null;
    }

    public Object getEndEffectorSubsystem() {
        return endEffectorSubsystem;
    }

    public void setEndEffectorSubsystem(Object value) {
        endEffectorSubsystem = value;
    }

    public boolean isEndEffectorSubsystemAvailable() {
        return endEffectorSubsystem != null;
    }

    public Object getFunnelSubsystem() {
        return funnelSubsystem;
    }

    public void setFunnelSubsystem(Object value) {
        funnelSubsystem = value;
    }

    public boolean isFunnelSubsystemAvailable() {
        return funnelSubsystem != null;
    }
}
