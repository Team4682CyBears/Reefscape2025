// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: EndEffectorSubsystem.java
// Intent: Subsystem for EndEffector which scores coral and clears algae
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.common.ToFDetector;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

public class EndEffectorSubsystem extends SubsystemBase {
    private TalonFX eeMotor = new TalonFX(Constants.eeMotorCanID);
    private final DutyCycleOut eeDutyCycle = new DutyCycleOut(0.0);

    private ToFDetector tofLeft;
    private ToFDetector tofRight;

    private EndEffectorDirection eeDirection = EndEffectorDirection.CORAL;
    private EndEffectorSpeed eeSpeed = EndEffectorSpeed.STOPPED;

    // Speeds between [-1, 1]
    private final double algaeSpeedFractional = 0.2;
    private final double handoffSpeedFractional = 0.3;
    private final double scoringSpeedFractional = 0.4;

    private final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;

    /**
     * Creates a new EndEffectorSubsystem.
     * Initializes the ToF sensors and configures the motor with default settings.
     */
    public EndEffectorSubsystem() {
        if (InstalledHardware.endEffectorTofsInstalled) {
            tofLeft = new ToFDetector(Constants.eeTofLeftCanID, Constants.eeTofDetectionThresholdInches);
            tofRight = new ToFDetector(Constants.eeTofRightCanID, Constants.eeTofDetectionThresholdInches);
        }
        configureMotor();
    }

    /**
     * Called periodically by the command scheduler.
     */
    public void periodic() {
        SmartDashboard.putBoolean("TOF DETECTS OBJECT", isCoralDetected());
        if (eeSpeed == EndEffectorSpeed.STOPPED) {
            eeMotor.stopMotor();
            return;
        }
        double motorSpeed;
        switch (eeSpeed) {
            case ALGAE:
                motorSpeed = algaeSpeedFractional;
                break;
            case HANDOFF:
                motorSpeed = handoffSpeedFractional;
                break;
            case SCORING:
                motorSpeed = scoringSpeedFractional;
                break;
            default:
                // Only happens when eeSpeed is invalid
                return;
        }

        int scalar = 1;

        if (eeDirection == EndEffectorDirection.ALGAE) {
            scalar = -1;
        }

        eeDutyCycle.withOutput(clamp(motorSpeed * scalar, -1, 1));
        eeMotor.setControl(eeDutyCycle);
    }

    /**
     * Stops the end effector motor and sets speed state to STOPPED.
     */
    public void stop() {
        eeSpeed = EndEffectorSpeed.STOPPED;
        eeMotor.stopMotor();
    }

    /**
     * Sets the direction of the end effector motion.
     * 
     * @param direction The desired direction (CORAL or ALGAE)
     */
    public void setDirection(EndEffectorDirection direction) {
        eeDirection = direction;
    }

    /**
     * Sets the speed mode of the end effector.
     * 
     * @param speed The desired speed mode (STOPPED, ALGAE, HANDOFF, or SCORING)
     */
    public void setSpeed(EndEffectorSpeed speed) {
        eeSpeed = speed;
    }

    /**
     * Checks if a coral is detected by any of the ToF sensors.
     * 
     * @return true if either ToF sensor detects an object within the threshold
     *         distance
     */
    public boolean isCoralDetected() {
        return (InstalledHardware.endEffectorTofsInstalled
                && (this.tofLeft.isDetected() || this.tofRight.isDetected()));
    }

    /**
     * Configures the TalonFX motor with settings for the end effector.
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        config.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        config.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = motorOutputInverted;

        StatusCode response = eeMotor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + eeMotor.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    // TODO when integrating into theRobot, use the clamp in MotorUtils.
    private double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(x, max));
    }
}
