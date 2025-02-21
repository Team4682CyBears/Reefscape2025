// ************************************************************
// Bishop Blanchet Robotics - Home of the Cybears
// FRC - Reefscape - 2025
// File: AlignWithBranchDirection.java
// Intent: File to store the state of which direction we want to align on the reef
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ• ʔ ʕ •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ• ʔ ʕ •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import frc.robot.common.AlignToBranchSide;

public class AlignWithBranchDirection {
    private AlignToBranchSide alignToBranchSide;

    public AlignWithBranchDirection(){
        alignToBranchSide = AlignToBranchSide.RIGHT;
    }

    public void setAlignWithBranchSide(AlignToBranchSide alignSide){
        this.alignToBranchSide = alignSide;
    }

    public AlignToBranchSide getAlignWithBranchSide(){
        return this.alignToBranchSide;
    }
}
