// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: EndEffectorSpeed.java
// Intent: Enum to hold speed for end effector
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

public enum EndEffectorSpeed {
    ALGAE, // TBD
    HANDOFF, // Slow (We do need to worry about loosing the piece)
    SCORING, // Fast (We don't need to worry about loosing the piece)
    STOPPED // Stopped
}