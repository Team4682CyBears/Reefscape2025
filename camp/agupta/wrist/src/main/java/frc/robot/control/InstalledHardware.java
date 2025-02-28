package frc.robot.control;

public class InstalledHardware {
    // Shooter Related Hardware
    public static final boolean shooterOutfeedInstalled = true;
    public static final boolean shooterAngleInstalled = true;
    // for testing, to decrease the power of the shooter angle mechanism, 
    // reduce the left motor gear box to 10x (instaed of 100x)
    // and disconnect the right motor from the chain. 
    public static final boolean shooterMotorInstalled = true;
    public static final boolean wristCanCoderInstalled = true; 
    // for testing shooter angle via the shuffleboard
    // ensure this is DISABLED for competitions, as it can sometimes crash shuffleboard
    public static final boolean setShooterAngleFromShuffleboard = false;
}