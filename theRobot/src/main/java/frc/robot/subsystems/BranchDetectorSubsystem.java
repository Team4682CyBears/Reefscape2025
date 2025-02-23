// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: BranchDetectorSubsystem.java
// Intent: Forms a subsystem to detect branches using ToFs.
// ************************************************************

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ToFDetector;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

/**
 * The BranchDetectorSubsystem class is responsible for detecting branches using ToFs
 */
public class BranchDetectorSubsystem extends SubsystemBase{
    
    ToFDetector tofLeft = new ToFDetector(Constants.tofLeftCanID, Constants.branchDetectionThresholdInches);
    ToFDetector tofRight = new ToFDetector(Constants.tofRightCanID, Constants.branchDetectionThresholdInches);

    /**
     * Constructs a BranchDetectorSubsystem object
     */
    public BranchDetectorSubsystem(){
    }
    
    /**
     * A method that returns true if either ToF detects something in range
     * @return - if somehthing detected by either ToF is in range
     */
    public boolean isBranchDetected(){
        boolean leftDetected = InstalledHardware.BranchTofLeft && tofLeft.isDetected();
        boolean rightDetected = InstalledHardware.BranchTofRight && tofRight.isDetected();

        return leftDetected || rightDetected;
    }
}
