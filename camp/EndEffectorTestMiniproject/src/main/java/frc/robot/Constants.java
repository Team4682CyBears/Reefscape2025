// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: Constants.java
// Intent: Contains constants for robot to run
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

public final class Constants {
  public static final int eeMotorCanId = 5;

  public static final int tofLeftCanId = 20;
  public static final int tofRightCanId = 21;
  public static final double tofDetectionThresholdInches = 20.0;

  public static final boolean leftTOFEnabled = true;
  public static final boolean rightTOFEnabled = true;

  public static final double motorStatorCurrentMaximumAmps = 100.0;

  public static final double motorSupplyCurrentMaximumAmps = 50.0;

  public static final int kDriverControllerPort = 0;
}
