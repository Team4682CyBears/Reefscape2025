package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.reefTofSensor;
import frc.robot.subsystems.Spinner;

public class RunExperimentCommand extends Command{
    Spinner spinMotor;
    reefTofSensor tofSensor;
    DoubleSupplier constSpeed;
    int cycles;
    Timer stopwatch = new Timer();
    boolean tofActivated = false;
    boolean previousTofActivation = false;
    double dataValue;
    int cyclesRun = 0;
    double[] dataValues;

    public RunExperimentCommand(DoubleSupplier constSpeed, Spinner spin, reefTofSensor tof, int cycles){
        Spinner spinMotor = spin;
        tofSensor = tof;
        this.constSpeed = constSpeed;
        this.cycles = cycles;
        dataValues = new double[cycles];
    }
    @Override
    public void execute(){
        spinMotor.spinMotor(constSpeed.getAsDouble()); // FIIIIIXXXXXX doubleSupplier -> double
        if(spinMotor.getSpeed() == constSpeed.getAsDouble()){
            stopwatch.start();
            tofActivated = tofSensor.isCoralDetected();
            if(tofActivated == true && previousTofActivation == false){
                dataValues[cyclesRun] = stopwatch.get();
                cycles -= 1;
                cyclesRun += 1;
            }
            previousTofActivation = tofActivated;
        }
        // prints out final data
        if(cycles == 0){
            spinMotor.motorStop();
            double meanValue = 0;
            double[] varianceData = new double[cyclesRun];
            for(int i = 1; i < cyclesRun; cyclesRun++){
                double x = dataValues[i] - dataValues[i-1];
                System.out.println(x);
                meanValue += x;
                varianceData[i-1] = x;
            }
            System.out.println("The speed of the experiment was: " + constSpeed.getAsDouble());
            System.out.println("Num Cycles run " + cyclesRun);
            System.out.println("The mean time between cycles was: " + meanValue);
            System.out.println("The variance was: " + calculateVariance(varianceData));
        }
    }

    public double calculateVariance(double[] data) {
        double sum = 0.0;
        double mean = 0.0;
        double variance = 0.0;
    
        // Calculate the mean
        for (double value : data) {
            sum += value;
        }
        mean = sum / data.length;
    
        // Calculate the variance
        for (double value : data) {
            variance += Math.pow(value - mean, 2);
        }
        variance /= data.length; // For population variance
    
        return variance;
    }

}
