package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.SubsystemCollection;

public class funnelClimbMotorTime extends Command{
    double time;
    Object movingSubsystem;
    boolean motorIsRunning = false;
    Timer motorTimer = new Timer();

    public funnelClimbMotorTime(double runTime, boolean isFunnel){
        time = runTime;
        if(isFunnel){
            movingSubsystem = SubsystemCollection.getFunnelSubsystem();
        }else{
            movingSubsystem = SubsystemCollection.getClimberSubsystem();
        }
    }

    public void intialize(){
        motorIsRunning = true;
        motorTimer.start();
    }

    public void execute(){
        if(motorTimer.getTimestamp() >= time){
            motorIsRunning = false;
            motorTimer.stop();
            //TODO: Motor should stop here
        }
        //TODO: Motor should run here
    }


}
