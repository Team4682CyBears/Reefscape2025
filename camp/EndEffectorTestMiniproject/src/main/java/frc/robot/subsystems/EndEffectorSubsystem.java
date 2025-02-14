package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.EndEffectorDirection;
import frc.robot.common.EndEffectorSpeed;
import frc.robot.common.ToFDetector;

public class EndEffectorSubsystem extends SubsystemBase {
    private TalonFX eeMotor = new TalonFX(Constants.eeMotorCANID);
    private final DutyCycleOut eeJoystickController = new DutyCycleOut(0.0);

    private final ToFDetector tofLeft;
    private ToFDetector tofRight;

    private EndEffectorDirection eeDirection = EndEffectorDirection.CORAL;
    private EndEffectorSpeed eeSpeed = EndEffectorSpeed.STOPPED;

    private final double algaeSpeed = 0.05;
    private final double handoffSpeed = 0.05;
    private final double scoringSpeed = 0.05;

    public EndEffectorSubsystem() {
        tofLeft = new ToFDetector(Constants.tofLeftCanID, Constants.tofDetectionThresholdInches);
        if (Constants.doubleTOF) {
            tofRight = new ToFDetector(Constants.tofRightCanID, Constants.tofDetectionThresholdInches);
        }
        configureMotor();
    }

    public void periodic() {
        if (eeSpeed == EndEffectorSpeed.STOPPED) {
            eeMotor.stopMotor();
            return;
        }
        double motorSpeed;
        switch (eeSpeed) {
            case ALGAE:
                motorSpeed = algaeSpeed;
                break;
            case HANDOFF:
                motorSpeed = handoffSpeed;
                break;
            case SCORING:
                motorSpeed = scoringSpeed;
                break;
            default:
                // Only happens when eeSpeed is invalid
                return;
        }

        // TODO:
        if (eeDirection == EndEffectorDirection.ALGAE) {
            motorSpeed = -motorSpeed;
        }

        eeJoystickController.withOutput(motorSpeed);
        eeMotor.setControl(eeJoystickController);
    }

    public void stop() {
        eeSpeed = EndEffectorSpeed.STOPPED;
        eeMotor.stopMotor();
    }

    public void setDirection(EndEffectorDirection direction) {
        eeDirection = direction;
    }

    public void setSpeed(EndEffectorSpeed speed) {
        eeSpeed = speed;
    }

    public boolean isBranchDetected() {
        return tofLeft.isDetected() || (Constants.doubleTOF && tofRight.isDetected());
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
        config.Voltage.SupplyVoltageTimeConstant = 0.02;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
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
