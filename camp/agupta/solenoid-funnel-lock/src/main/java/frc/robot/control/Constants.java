// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

package frc.robot.control;

public class Constants {
    // *****************************************************************
    // input device constants
    public static final int portDriverController = 0;

    // ******************************************************************
    // solenoid constants
    public static final int solenoidCanId = 15;
    // solenoidSpeed is [0.0 .. 1.0]
    public static final double solenoidSpeed = 1.0;
    public static final double solenoidDuration = 2.0;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
    
}
