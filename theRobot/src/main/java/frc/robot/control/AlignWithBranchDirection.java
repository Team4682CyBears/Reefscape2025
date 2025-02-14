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
