package frc.robot.control;

public class InstalledHardware {
    // Wrist Related Hardware
    public static final boolean wristOutfeedInstalled = true;
    public static final boolean coralAngleInstalled = true;
    // for testing, to decrease the power of the wrist angle mechanism, 
    // reduce the left motor gear box to 10x (instaed of 100x)
    // and disconnect the right motor from the chain. 
    public static final boolean wristMotorInstalled = true;
    public static final boolean wristCanCoderInstalled = true; 
    // for testing wrist angle via the shuffleboard
    // ensure this is DISABLED for competitions, as it can sometimes crash shuffleboard
    public static final boolean setcoralAngleFromShuffleboard = false;
}