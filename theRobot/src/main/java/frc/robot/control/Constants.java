// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// imports for shooter angle
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


public final class Constants {

    public final static double DriveVoltageScalar = 1.0;

    //////// SWERVE MODULE CONFIGS ///////////
    public static final double SWERVE_MAX_SPEED = 4.3251; // m/s
    public static final double SWERVE_MAX_ANGULAR_SPEED = 405; // rad/s

    // *****************************************************************
    // Auto Constants
    public static final PPHolonomicDriveController pathFollower = new PPHolonomicDriveController(
        new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(4.5, 0.001, 0.0) // Rotation PID constants 
    );

    // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // NEO 550 maximum RPM - see:
    // https://www.revrobotics.com/rev-21-1651/#:~:text=The%20following%20specifications%20for%20the%20NEO%20550%20Brushless,Motor%20Kv%3A%20917%20Kv%20Free%20Speed%3A%2011000%20RPM
    public static final double neoFiveFiveZeroMaximumRevolutionsPerMinute = 11000;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per
    // rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42;
    // CTRE motor constants
    public static final double talonMaximumRevolutionsPerMinute = 6380;

    // *****************************************************************
    // input device constants
    public static final int portDriverController = 0;
    public static final int portCoDriverController = 1;

    // ******************************************************************
    // Branch Detector constants
    public static final double branchDetectionThresholdInches = 20.0;

    // ******************************************************************
    // led constants
    public static final int ledCanID = 24;
    public static final int ledLength = 72;
    public static final int ledStartIdx = 0;
    public static final int ledBlinkFrquencyInHertz = 2;
    public static final double ledBrightness = 0.5;
    public static final LEDStripType ledStripType = LEDStripType.RGB;
    public static final int tofLeftCanID = 20;
    public static final int tofRightCanID = 21;

    // ******************************************************************
    // camera constants
    public static final boolean useFusedVisionInAuto = false;
    public static final double autoUseFusedVisionDuration = 15.0;

    // Distance from center of the robot to the reef tag.
    public static final double alignDistanceFromReefMeters = 0.65;

    public static final double limelightToWPIBlueXOffest = 8.75;
    public static final double limelightToWPIBlueYOffset = 4.0;
  
    // ********************************************************************
    // Controller Constants
    public static final double rumbleTimeSeconds = 0.15;

    // ********************************************************************
    // PowerDistributionPanel Constants
    public static final int currentPowerDistributionPanelCanId = 29;
    public static final ModuleType currentPowerDistributionPanelType = ModuleType.kRev;
    public static final double overcurrentRumbleTimeSeconds = 0.25;

    // ********************************************************************
    // Diagnostic Constants
    public static final boolean putDiagnosticPaths = false;

    // ********************************************************************
    // CAN IDs

    // Funnel
    public static final int funnelTofCanID = 16;

    // Elevator
    public static final int elevatorMotorLeftCanID = 14;
    public static final int elevatorMotorRightCanID = 15;

    // Wrist
    public static final int wristMotorCanID = 17;
    public static final int wristCANCoderCanID = 18;

    // End Effector
    public static final int eeMotorCanID = 19;
    public static final int eeTofLeftCanID = 20;
    public static final int eeTofRightCanID = 21;

    // Climber
    public static final int climberMotorCanID = 22;
    public static final int funnelMotorCanID = 23;

    // Wrist
    public static final int shooterMotorCanId = 5;
    public static final int shooterEncoderCanId = 24;

    // ********************************************************************
    // Wrist
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    // public static final double shooterAngleShootFromSpeaker = 54.5;
    public static final double shooterAngle = 0;
    public static final double algaeAngle = 90;

    // MotionMagic 
    public static final double cruiseVelocity = 800.0;
    public static final double acceleration = 160;
    public static final double jerk = 800; 

    //Motor angle min/max
    public static final double shooterAngleMaxDegrees = 110;
    public static final double shooterAngleMinDegrees = -20;  

    public static InvertedValue angleTalonShooterMotorDefaultDirection = InvertedValue.Clockwise_Positive;
    public static final double shooterStartingAngleOffsetDegrees = 20.0;

    // *******************************************************************
    // shooter angle constants 
    public static SensorDirectionValue shooterAngleSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final double shooterAbsoluteAngleOffsetDegrees = -166.11;
    public static final double shooterAngleToleranceDegrees = 0.5;
}