// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: Constants.java
// Intent: Separate class to store constants used across subsystem
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }

  // Can IDs
  public static final int wristMotorCanId = 5;
  public static final int wristEncoderCanId = 24;

  public static final double coralAngle = 0;
  public static final double algaeAngle = 90;

  // MotionMagic 
  public static final double cruiseVelocity = 800.0;
  public static final double acceleration = 160;
  public static final double jerk = 800; 

  //Motor angle min/max
  public static final double wristMaxDegrees = 110;
  public static final double wristMinDegrees = -20;  

  public static InvertedValue angleTalonWristMotorDefaultDirection = InvertedValue.Clockwise_Positive;
  public static final double wristStartingAngleOffsetDegrees = 20.0;

  // *******************************************************************
  // wrist angle constants 
  public static SensorDirectionValue wristSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
  public static final double wristAbsoluteAngleOffsetDegrees = -166.11;
  public static final double wristToleranceDegrees = 0.5;

}
