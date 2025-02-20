package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ToFDetector;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

public class BranchDetectorSubsystem extends SubsystemBase{
    
    ToFDetector tofLeft = new ToFDetector(Constants.tofLeftCanID, Constants.branchDetectionThresholdInches);
    ToFDetector tofRight = new ToFDetector(Constants.tofRightCanID, Constants.branchDetectionThresholdInches);

    public BranchDetectorSubsystem(){

    }

    public void periodic(){
    }

    /**
     * A method that returns true if either ToF detects something in range
     * @return - if somehthing detected by either ToF is in range
     */
    public boolean isBranchDetected(){
        boolean leftDetected = InstalledHardware.EEToFLeft && tofLeft.isDetected();
        boolean rightDetected = InstalledHardware.EEToFRight && tofRight.isDetected();

        return leftDetected || rightDetected;
    }
}
