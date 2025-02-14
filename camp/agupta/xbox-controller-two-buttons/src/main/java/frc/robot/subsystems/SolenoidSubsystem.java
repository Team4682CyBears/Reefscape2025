// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: SolenoidSubsystem.java
// Intent: Forms the prelminary code for solenoid subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// declare package containing class
package frc.robot.subsystems;

// import motor controller libraries
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

// import wpi libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import local classes
import frc.robot.control.Constants;
import frc.robot.common.SolenoidMode;

/**
 * Forms a class for the Solenoid subsystem
 */
public class SolenoidSubsystem extends SubsystemBase {
  // create new instance of solenoid controller class
  private TalonSRX solenoid = new TalonSRX(Constants.solenoidCanId);
  // vendordeps library:
  // https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json

  // Direction Mode default is Push
  SolenoidMode solenoidMode = SolenoidMode.Push;
  private int pull = -1; // set 1 for not inverted, set -1 for inverted
  private int push = 1; // set 1 for not inverted, set -1 for inverted

  /**
   * Constructor for the IntakeSubsystem
   */
  public SolenoidSubsystem() {
    solenoid.setNeutralMode(NeutralMode.Brake); // set motor off

    // set configuration
    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.html
    TalonSRXConfiguration config = new TalonSRXConfiguration(); // create a new config instance
    config.peakCurrentLimit = 20; // the peak current, in amps
    config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
    config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
    solenoid.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
  }

  /**
   * A method to set the solenoid mode
   * @param solenoidMode
   */
  public void setSolenoidMode(SolenoidMode solenoidMode) {
    // If the solenoid mode is changing, stop the solenoid
    // this prevents the solenoid from going from +1 to -1 abruptly
    if (solenoidMode != this.solenoidMode){
      setAllStop();
    }
    this.solenoidMode = solenoidMode;
  }

  /**
   * A method to stop the intake subsystem
   */
  public void setAllStop() {
    solenoid.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /**
   * A method to set the motor to the given percent in the 
   * direction of the shooter or solenoid depending on the mode
   * @param percent (between 0 and 1)
   */
  public void setSolenoidSpeed(double percent) {

    // change solenoid direction based of solenoid mode shooter or solenoid
    int direction = solenoidMode == SolenoidMode.Push ? push : pull; 

    // set solenoid speed
    solenoid.set(TalonSRXControlMode.PercentOutput, percent * direction);
  }



    
}


