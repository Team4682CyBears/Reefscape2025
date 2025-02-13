package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.Timer;

public class StopCommandAfterDelayCommand extends Command {

    Timer timer = new Timer();
    double delaySeconds;
    Command commandToStop;

    boolean done = false;

    public StopCommandAfterDelayCommand(double delaySeconds, Command commandToStop) {
        this.delaySeconds = delaySeconds;
        this.commandToStop = commandToStop;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
        System.out.println("I AM MAKING THIS COMMAND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    @Override
    public void execute() {
        if(timer.hasElapsed(delaySeconds)){
            CommandScheduler.getInstance().cancel(commandToStop);
            System.out.println("I AM if statment THIS COMMAND??????????????????????????????????????????????///");
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            CommandScheduler.getInstance().cancel(commandToStop);
            done = true;
        }
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}