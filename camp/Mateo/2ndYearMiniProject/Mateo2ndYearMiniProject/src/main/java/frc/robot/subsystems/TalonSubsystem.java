package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TalonSubsystem extends SubsystemBase {
    public double speed = 0;
    private TalonFX talon = new TalonFX(0);

    public TalonSubsystem() {
        //configure stuff here
    }

    public void decreaseSpeed() {
        talon.set(speed - Constants.RATEOFCHANGE);
    }

    public void increaseSpeed() {
        talon.set(speed + Constants.RATEOFCHANGE);
    }

    @Override
    public void simulationPeriodic() {
    }
}
