// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: FeederSubsystem.java
// Intent: Forms a command to run the solenoid
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// declare package to contain current class
package frc.robot.commands;

// import wpi libraries
//import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

// import local classes
import frc.robot.common.SolenoidMode;
import frc.robot.control.Constants;
import frc.robot.subsystems.SolenoidSubsystem;


public class RunSolenoidCommand extends Command
{
  private SolenoidSubsystem solenoid;
  private SolenoidMode direction; 
  private boolean done = false;

  private Timer timer = new Timer();
 
  /** 
  * Makes solenoid push
  * @param solenoidSubsystem - the solenoid subsystem
  * @param solenoidMode - the direction for the solenoid
  */

  public RunSolenoidCommand(SolenoidSubsystem solenoidSubsystem, SolenoidMode solenoidMode)
  {
    this.solenoid = solenoidSubsystem;
    this.direction = solenoidMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(solenoid);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    solenoid.setSolenoidMode(direction); // set direction (push or pull)
    solenoid.setSolenoidSpeed(Constants.solenoidSpeed); // run solenoid
    done = false;

    timer.start();
    System.out.println("Timer Started");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(timer.isRunning() ==false){
      timer.reset();
    }
    solenoid.setSolenoidMode(direction); // set direction (push or pull)
    solenoid.setSolenoidSpeed(Constants.solenoidSpeed); // run solenoid
    if (timer.get() >= 2){
      done = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(timer.get());
    timer.reset();
    timer.stop();
    solenoid.setAllStop();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }

}
