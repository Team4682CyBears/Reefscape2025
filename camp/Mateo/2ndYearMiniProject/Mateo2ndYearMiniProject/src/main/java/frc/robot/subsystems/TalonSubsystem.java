package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TalonSubsystem extends SubsystemBase {
    public double speed = 0;
    private TalonFX talon = new TalonFX(Constants.talonID);
    private DutyCycleOut tDutyCycle = new DutyCycleOut(0.0);
    private final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;

    public TalonSubsystem() {
        configureMotor();
    }

    public void decreaseSpeed() {
        System.out.println("!!!!!!!!!!DECREASE SPEED!!!!!!!!!!!!");
        //TODO need to clamp the speed to +/- 1
        speed -= Constants.RATEOFCHANGE;
    }

    public void increaseSpeed() {
        System.out.println("!!!!!!!!!!INCREASE SPEED!!!!!!!!!!!!");
                //TODO need to clamp the speed to +/- 1
                // use MathUtil.clamp?
        speed += Constants.RATEOFCHANGE;
    }

    public void periodic(){

        tDutyCycle.withOutput(speed);
        talon.setControl(tDutyCycle);
    }

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

        config.MotorOutput.Inverted = motorOutputInverted;

        StatusCode response = talon.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + talon.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}
