package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ToFDetector;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

public class EndEffectorSubsystem extends SubsystemBase{
    
    ToFDetector tofLeft = new ToFDetector(Constants.tofLeftCanID, Constants.branchDetectionThresholdInches);
    ToFDetector tofRight = new ToFDetector(Constants.tofRightCanID, Constants.branchDetectionThresholdInches);

    public EndEffectorSubsystem(){

    }

    public boolean isBranchDetected(){
        boolean leftDetected = InstalledHardware.EEToFLeft && tofLeft.isDetected();
        boolean rightDetected = InstalledHardware.EEToFRight && tofRight.isDetected();

        return leftDetected || rightDetected;
    }
}
