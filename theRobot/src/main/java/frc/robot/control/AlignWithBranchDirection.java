// ************************************************************
// Bishop Blanchet Robotics - Home of the Cybears
// FRC - Reefscape - 2025
// File: AlignWithBranchDirection.java
// Intent: File to store the state of which direction we want to align on the reef
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ• ʔ ʕ •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ• ʔ ʕ •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import frc.robot.common.AlignToBranchSide;

/**
 * The AlignWithBranchDirection class is responsible for stroring a specific
 * branch direction.
 * It uses the AlignToBranchSide enum to determine the side to align with.
 */
public class AlignWithBranchDirection {

    /**
     * Enumeration representing the side to align with.
     */
    private AlignToBranchSide alignToBranchSide;

    /**
     * Constructs an AlignWithBranchDirection object and sets the default alignment
     * side to RIGHT.
     */
    public AlignWithBranchDirection() {
        alignToBranchSide = AlignToBranchSide.RIGHT;
    }

    /**
     * Sets the alignment side.
     *
     * @param alignSide the side to align with, represented by the AlignToBranchSide
     *                  enum.
     */
    public void setAlignWithBranchSide(AlignToBranchSide alignSide) {
        this.alignToBranchSide = alignSide;
    }

    /**
     * Gets the current alignment side.
     *
     * @return the current alignment side, represented by the AlignToBranchSide
     *         enumeration.
     */
    public AlignToBranchSide getAlignWithBranchSide() {
        return this.alignToBranchSide;
    }
}
