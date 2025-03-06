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
    
    ToFDetector tofLeft = new ToFDetector(Constants.branchDetectorTofLeftCanID, Constants.branchDetectionThresholdInches, Constants.minimumBranchDetectionThresholdInches);
    ToFDetector tofRight = new ToFDetector(Constants.branchDetectorTofRightCanID, Constants.branchDetectionThresholdInches, Constants.minimumBranchDetectionThresholdInches);

    /**
     * Constructs a BranchDetectorSubsystem object
     */
    public BranchDetectorSubsystem(){
    }

    @Override
    public void periodic(){
        tofLeft.publishTelemetery();
        tofRight.publishTelemetery();
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
