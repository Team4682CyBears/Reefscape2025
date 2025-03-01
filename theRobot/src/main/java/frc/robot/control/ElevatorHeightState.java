// ************************************************************
// Bishop Blanchet Robotics - Home of the Cybears
// FRC - Reefscape - 2025
// File: ElevatorHeightState.java
// Intent: File to store the state of which height we want the elevator
// this class does not move the elevator, but just saves the desired state for 
// when a command is later called to move the elevator to that state.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ• ʔ ʕ •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ• ʔ ʕ •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import frc.robot.common.ElevatorPositions;

/**
 * The ElevatorHeightState class is responsible for storing the desired height of the elevator.
 */
public class ElevatorHeightState {
    ElevatorPositions elevatorPosition = ElevatorPositions.STOW;

    /**
     * Constructs an ElevatorHeightState object and sets the default elevator position to STOW.
     */
    public ElevatorHeightState() {
    }
    
    /**
     * Sets the desired elevator position.
     * @return
     */
    public ElevatorPositions gElevatorPosition() {
        return elevatorPosition;
    }

    /**
     * Gets the desired elevator position.
     * @param elevatorPosition
     */
    public void sElevatorPosition(ElevatorPositions elevatorPosition) {
        this.elevatorPosition = elevatorPosition;
    }
}
