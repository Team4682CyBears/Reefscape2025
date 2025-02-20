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

import com.ctre.phoenix.led.CANdle.LEDStripType;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;

    //////// SWERVE MODULE CONFIGS ///////////
    public static final double SWERVE_MAX_SPEED = 4.3251; // m/s
    public static final double SWERVE_MAX_ANGULAR_SPEED = 405; // rad/s

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

    // ******************************************************************
    // camera constants
    public static final boolean useFusedVisionInAuto = false;
    public static final double autoUseFusedVisionDuration = 15.0;
  
    // ********************************************************************
    // Controller Constants
    public static final double rumbleTimeSeconds = 0.15;

    // ********************************************************************
    // PowerDistributionPanel Constants
    public static final int currentPowerDistributionPanelCanId = 29;
    public static final ModuleType currentPowerDistributionPanelType = ModuleType.kRev;
    public static final double overcurrentRumbleTimeSeconds = 0.25;

}