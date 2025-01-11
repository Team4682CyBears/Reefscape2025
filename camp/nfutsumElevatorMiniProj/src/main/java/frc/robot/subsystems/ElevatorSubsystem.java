// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ElevatorSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.MotorUtils;
import java.util.*;

//import javax.lang.model.util.ElementScanner14;

public class ElevatorSubsystem extends SubsystemBase
{
  private TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorCANID);
  private final MotionMagicExpoDutyCycle angleLeftVoltageController = new MotionMagicExpoDutyCycle(0);

  private Slot0Configs angleMotorGainsForInternalEncoder = new Slot0Configs().withKP(350).withKI(0).withKD(50.0).withKV(0);
  private Slot0Configs angleMotorGainsForAbsoluteEncoder = new Slot0Configs().withKP(150).withKI(0.125).withKD(0.05).withKV(0);
}