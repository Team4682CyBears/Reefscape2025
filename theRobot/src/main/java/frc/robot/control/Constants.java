// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.common.DrivetrainSwerveConfig;

import com.ctre.phoenix.led.CANdle.LEDStripType;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;

    //////////////////// FOO DRIVETRAIN ////////////////////
    public static final DrivetrainSwerveConfig fooDrivetrainConfig = new DrivetrainSwerveConfig(
        Units.inchesToMeters(28), 
        Units.inchesToMeters(28), 
        // SUBTRACT the values you find in shuffleboard
        Math.toRadians(0), // FRONT LEFT
        Math.toRadians(0), // FRONT RIGHT 
        Math.toRadians(0), // BACK LEFT
        Math.toRadians(0)); // BACK RIGHT

    //////////////////// TED DRIVETRAIN ////////////////////
    public static final DrivetrainSwerveConfig tedDrivertainConfig = new DrivetrainSwerveConfig(
        Units.inchesToMeters(23.25), 
        Units.inchesToMeters(22.75), 
        // SUBTRACT the values you find in shuffleboard
        //Math.toRadians(-215.15 - 178.76 - 358.7 - 43.9 - 354.5), // FRONT LEFT
        //Math.toRadians(-180.61 - 95.27 - 358.6 -169.8 - 150.59), // FRONT RIGHT 
        //Math.toRadians(-191.33 - 257.52 -357.3  - 3.6 - 355.67 - 230.9 - 180), // BACK LEFT
        //Math.toRadians(-58.35 - 177.27 - 2.0 - 358.50 - 51.9 - 136)); // BACK RIGHT
        Math.toRadians(0), // FRONT LEFT
        Math.toRadians(0), // FRONT RIGHT 
        Math.toRadians(0), // BACK LEFT
        Math.toRadians(0)); // BACK RIGHT
        

    //////// SWERVE MODULE CONFIGS ///////////
    public static final double SWERVE_MAX_SPEED = 4.3251; // m/s
    public static final double SWERVE_WHEEL_DIAMETER = 0.1016;  //4" in meters
    public static final double SWERVE_DRIVE_REDUCTION = 1/5.46; // 5.46:1 (12:34 -> 28:18 -> 15:45)
    public static final boolean SWERVE_DRIVE_INVERTED = true; //three reductions
    public static final double SWERVE_STEER_REDUCTION = 1/15.43; // 15.43:1 (8:24 -> 14:72)
    public static final boolean SWERVE_STEER_INVERTED = false; //two reductions

    // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM 
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // NEO 550 maximum RPM - see: https://www.revrobotics.com/rev-21-1651/#:~:text=The%20following%20specifications%20for%20the%20NEO%20550%20Brushless,Motor%20Kv%3A%20917%20Kv%20Free%20Speed%3A%2011000%20RPM
    public static final double neoFiveFiveZeroMaximumRevolutionsPerMinute = 11000;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42;
    // CTRE motor constants
    public static final double talonMaximumRevolutionsPerMinute = 6380;
    public static final double CtreTalonFx500EncoderTicksPerRevolution = 2048; 

    // *****************************************************************
    // input device constants
    public static final int portDriverController = 0;
    public static final int portCoDriverController = 1;

    // ******************************************************************
    //led constants
    public static final int ledCanID = 30;
    public static final int ledLength = 72;
    public static final int ledStartIdx = 0;
    public static final int ledBlinkFrquencyInHertz = 2; 
    public static final double ledBrightness = 0.5; 
    public static final LEDStripType ledStripType = LEDStripType.RGB;
    public static final int tofLeftCanID = 21;
    public static final int tofRightCanID = 22;

    // ******************************************************************
    // camera constants
    public static final boolean useFusedVisionInAuto = false;
    public static final double autoUseFusedVisionDuration = 15.0;

    public static final double alignDistanceFromReef = .75;
  
    // ********************************************************************
    // Controller Constants
    public static final double rumbleTimeSeconds = 0.15;

    // ********************************************************************
    // PowerDistributionPanel Constants
    public static final int currentPowerDistributionPanelCanId = 29;
    public static final ModuleType currentPowerDistributionPanelType = ModuleType.kRev;
    public static final double overcurrentRumbleTimeSeconds = 0.25;

}