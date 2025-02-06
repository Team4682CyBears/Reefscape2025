// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;

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

  public static final int elevatorMotorCANID = 5;
  public static final int elevatorMageneticSensorID = 0;

  public static final Distance stowHeight = Inches.of(0.0);
  public static final Distance L2Height = Inches.of(32.0);;
  public static final Distance L3Height = Inches.of(48.0);
  public static final Distance L4Height = Inches.of(72.0);


  // Motor Config constant variables
  public static final int kDriverControllerPort = 0;
  public static final double motorStatorCurrentMaximumAmps = 100.0;
  public static final double motorSupplyCurrentMaximumAmps = 50.0;
  public static final double motorSupplyVoltageTimeConstant = 0.02;
  // Motor Talon Port
  public static final int motorCanID = 3;
  public static final int secondMotorCanID = 4;
  public static final double falconMaxVoltage = 12.0;
  public static final int falconFreeSpeedRps = 6380 / 60;
  public static final double minimumMotorSpeedRpm = 0.25 * 60;

}
