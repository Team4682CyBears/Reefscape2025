// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ElevatorPositions.java
// Intent: define 4 positions the robot can move to
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// add class to namespace
package frc.robot.common;

// positions from lowest to highest on reef
public enum ElevatorPositions {
    STOW, // A middle state to more quickly move from one resting to another position
    L1,
    L2,
    L3,
    L4,
    SENSE
}
