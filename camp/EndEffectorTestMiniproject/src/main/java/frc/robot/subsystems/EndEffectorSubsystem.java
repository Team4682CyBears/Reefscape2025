// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: EndEffectorSubsystem.java
// Intent: Subsystem for EndEffector to score coral
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
import frc.robot.Constants;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.common.ToFDetector;

public class EndEffectorSubsystem extends SubsystemBase {
    private TalonFX eeMotor = new TalonFX(Constants.eeMotorCanId);
    private final DutyCycleOut eeJoystickController = new DutyCycleOut(0.0);

    private ToFDetector tofLeft;
    private ToFDetector tofRight;

    private EndEffectorDirection eeDirection = EndEffectorDirection.CORAL;
    private EndEffectorSpeed eeSpeed = EndEffectorSpeed.STOPPED;

    private final double algaeSpeed = 0.2;
    private final double handoffSpeed = 0.3;
    private final double scoringSpeed = 0.4;

    /**
     * Creates a new EndEffectorSubsystem.
     * Initializes the ToF sensors and configures the motor with default settings.
     */
    public EndEffectorSubsystem() {
        if (Constants.leftTOFEnabled) {
            tofLeft = new ToFDetector(Constants.tofLeftCanId, Constants.tofDetectionThresholdInches);
        }
        if (Constants.rightTOFEnabled) {
            tofRight = new ToFDetector(Constants.tofRightCanId, Constants.tofDetectionThresholdInches);
        }
        configureMotor();
    }

    /**
     * Called periodically by the command scheduler.
     */
    public void periodic() {
        SmartDashboard.putBoolean("TOF DETECTS OBJECT", isBranchDetected());
        if (eeSpeed == EndEffectorSpeed.STOPPED) {
            eeMotor.stopMotor();
            return;
        }
        double motorSpeed;
        switch (eeSpeed) {
            case ALGAE:
                motorSpeed = SmartDashboard.getNumber("Algae Speed", algaeSpeed);
                break;
            case HANDOFF:
                motorSpeed = SmartDashboard.getNumber("Handoff Speed", handoffSpeed);
                break;
            case SCORING:
                motorSpeed = SmartDashboard.getNumber("Scoring Speed", scoringSpeed);
                break;
            default:
                // Only happens when eeSpeed is invalid
                return;
        }

        int scalar = 1;

        if (eeDirection == EndEffectorDirection.ALGAE) {
            scalar = -1;
        }

        eeJoystickController.withOutput(motorSpeed * scalar);
        eeMotor.setControl(eeJoystickController);
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
     * Checks if a coral branch is detected by any of the ToF sensors.
     * 
     * @return true if either ToF sensor detects an object within the threshold distance
     */
    public boolean isBranchDetected() {
        return (Constants.leftTOFEnabled && tofLeft.isDetected()) || (Constants.rightTOFEnabled && tofRight.isDetected());
    }

    /**
     * Configures the TalonFX motor with settings for the end effector.
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.Voltage.SupplyVoltageTimeConstant = 0.02;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode response = eeMotor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + eeMotor.getDeviceID() + " failed config with error " + response.toString());
        }
    }
}
