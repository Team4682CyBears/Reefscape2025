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
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.BranchDetectorSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SimpleNeoMotorSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class SubsystemCollection {
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems
    private CameraSubsystem cameraSubsystem = null;
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private BranchDetectorSubsystem branchDetectorSubsystem = null;
    private AlignWithBranchDirection alignWithBranchDirection = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null;
    private LEDSubsystem ledSubsystem = null;
    private ElevatorSubsystem elevatorSubsystem = null;
    private ElevatorHeightState elevatorHeightState = null;
    private EndEffectorSubsystem endEffectorSubsystem = null; 
    private SimpleNeoMotorSubsystem funnelSubsystem = null;
    private SimpleNeoMotorSubsystem climberSubsystem = null;

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
    public boolean isLEDSubsystemAvailable() { return ledSubsystem != null; }

    public AlignWithBranchDirection getAlignWithBranchDirection() { return alignWithBranchDirection; }
    public void setAlignWithBranchDirection(AlignWithBranchDirection value) { alignWithBranchDirection = value; }
    public boolean isAlignWithBranchDirection() { return alignWithBranchDirection != null; }

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

    public ElevatorSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }

    public void setElevatorSubsystem(ElevatorSubsystem value) {
        elevatorSubsystem = value;
    }

    public boolean isElevatorSubsystemAvailable() {
        return elevatorSubsystem != null;
    }

    public ElevatorHeightState getElevatorHeightState() {
        return elevatorHeightState;
    }

    public void setElevatorHeightState(ElevatorHeightState value) {
        elevatorHeightState = value;
    }

    public boolean isElevatorHeightStateAvailable() {
        return elevatorHeightState != null;
    }

    public EndEffectorSubsystem getEndEffectorSubsystem() {
        return endEffectorSubsystem;
    }

    public void setEndEffectorSubsystem(EndEffectorSubsystem value) {
        endEffectorSubsystem = value;
    }

    public boolean isEndEffectorSubsystemAvailable() {
        return endEffectorSubsystem != null;
    }

    public SimpleNeoMotorSubsystem getFunnelSubsystem() {
        return funnelSubsystem;
    }

    public void setFunnelSubsystem(SimpleNeoMotorSubsystem value) {
        funnelSubsystem = value;
    }

    public boolean isFunnelSubsystemAvailable() {
        return funnelSubsystem != null;
    }

    public SimpleNeoMotorSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public void setClimberSubsystem(SimpleNeoMotorSubsystem value) {
        climberSubsystem = value;
    }

    public boolean isClimberSubsystemAvailable() {
        return climberSubsystem != null;
    }
}
