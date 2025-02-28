// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.control;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // Can IDs
  public static final int shooterMotorCanId = 5;
  public static final int shooterEncoderCanId = 24;

  // public static final double shooterAngleShootFromSpeaker = 54.5;
  public static final double shooterAngle = 0;
  public static final double algaeAngle = 90;

  // MotionMagic 
  public static final double cruiseVelocity = 800.0;
  public static final double acceleration = 160;
  public static final double jerk = 800; 

  //Motor angle min/max
  public static final double wristMaxDegrees = 110;
  public static final double wristMinDegrees = -20;  

  public static InvertedValue angleTalonShooterMotorDefaultDirection = InvertedValue.Clockwise_Positive;
  public static final double shooterStartingAngleOffsetDegrees = 20.0;

  // *******************************************************************
  // shooter angle constants 
  public static SensorDirectionValue wristSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
  public static final double shooterAbsoluteAngleOffsetDegrees = -166.11;
  public static final double wristToleranceDegrees = 0.5;

}
