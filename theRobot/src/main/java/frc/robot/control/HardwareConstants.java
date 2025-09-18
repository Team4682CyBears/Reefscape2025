// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: HardwareConstants.java
// Intent: Forms key HARDWARE-only constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

public class HardwareConstants {

    // *******************************************************************
    // CTRE motor constants
    public static final double ctreStatorCurrentMaximumAmps = 100.0;
    public static final double ctreSupplyCurrentMaximumAmps = 50.0;
    public static final double ctreSupplyVoltageTimeConstant = 0.02;

    // ********************************************************************
    // CAN Optimization Constants
    // ctre
    public static final int ctreMotorStatusFramePeriodMilliseconds = 20; // 20ms refresh rate
    public static final int ctreSensorStatusFramePeriodMilliseconds = 30; // 30ms refresh rate
    public static final double ctreMotorStatusFramePeriodFrequencyHertz = 1000.0
            / (double) HardwareConstants.ctreMotorStatusFramePeriodMilliseconds;
    public static final double ctreSensorStatusFramePeriodFrequencyHertz = 1000.0
            / (double) HardwareConstants.ctreSensorStatusFramePeriodMilliseconds;
    // playing with fusion
    public static final double playingWithFusionSensorPeriodMilliseconds = 24; // 24ms refresh rate as specified in
                                                                               // their documentation (max refresh rate)

    // ********************************************************************
    // Wrist
   
    // *******************************************************************
    // shooter outfeed motor constants
    public static final double shooterOutfeedStatorCurrentMaximumAmps = HardwareConstants.ctreStatorCurrentMaximumAmps;
    public static final double shooterOutfeedSupplyCurrentMaximumAmps = HardwareConstants.ctreSupplyCurrentMaximumAmps;
    public static final double shooterOutfeedSupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;
      
    // *******************************************************************
    // shooter angle motor constants
    public static final double wristStatorCurrentMaximumAmps = HardwareConstants.ctreStatorCurrentMaximumAmps;
    public static final double wristSupplyCurrentMaximumAmps = HardwareConstants.ctreSupplyCurrentMaximumAmps;
    public static final double wristSupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;

}
