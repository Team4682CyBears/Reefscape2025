// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: InstalledHardware.java
// Intent: Forms a listing of switches that will help to debug code better as hardware is available (or not available).
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

/**
 * A class devoted to installed hardware constants. Use this to decide if
 * hardware is enabled on the robot or not.
 * All developers to use this to protect their subsystems and commands from
 * using hardware that is not actually installed
 * on the robot at hand. Used to assist in development stages and make it easier
 * to quickly remove a piece of hardware
 * from the robot.
 */
public class InstalledHardware {
    // Basic Hardware
    public static final boolean powerDistributionPanelInstalled = true;

    // Onboard Hardware - Orentation/Navigation Hardware
    public static final boolean wifiRadioInstalled = true;
    public static final boolean limelightInstalled = true;

    // External Input Hardware
    public static final boolean driverXboxControllerInstalled = true;
    public static final boolean coDriverXboxControllerInstalled = true;

    // DriveTrain Related Hardware
    public static final boolean drivetrainInstalled = true;
    public static final boolean tardiDrivetrainInstalled = true;

    // TOF Sensor Hardware
    // Important! You must disable any TOF sensor that is not installed!!
    // If you try to configure a TOF sensor that is not installed
    // the other TOF sensors that are installed will not work.
    public static final boolean BranchTofLeft = true;
    public static final boolean BranchTofRight = true;
    // this one controls the install of both ee TOFs.
    public static final boolean handoffTofsInstalled = true;

    // LED Hardware
    public static final boolean LEDSInstalled = true;

    // Branch Detector
    public static final boolean branchDetectorInstalled = true;

    // EndEffector
    public static final boolean endEffectorInstalled = true;

    // Elevator
    public static final boolean elevatorInstalled = true;

    // Funnel
    public static final boolean funnelInstalled = true;

    // Climber
    public static final boolean climberInstalled = true;
}
