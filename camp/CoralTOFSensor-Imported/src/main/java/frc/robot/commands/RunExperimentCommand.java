package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.reefTofSensor;
import frc.robot.subsystems.Spinner;

public class RunExperimentCommand extends Command {
    Spinner spiningMotor;
    reefTofSensor tofSensor;
    double constSpeed;
    double desiredSpeed = 0;
    boolean isDone = false;
    int cycles;
    Timer stopwatch = new Timer();
    boolean tofActivated = false;
    boolean previousTofActivation = false;
    double dataValue;
    int cyclesRun = 0;
    double[] dataValues;
    boolean experimentRunning = false;

    public RunExperimentCommand(Double constS, Spinner spin, reefTofSensor tof, int cyc) {
        spiningMotor = spin;
        tofSensor = tof;
        constSpeed = constS;
        cycles = cyc;
        dataValues = new double[cycles];
    }

    @Override
    public void initialize() {
        isDone = false;
        cyclesRun = 0;
        // try to reset the controller to ensure behavior is consistent
        spiningMotor.resetController();
        spiningMotor.spinMotor(desiredSpeed);
        stopwatch.reset();
        desiredSpeed = constSpeed/60;
        experimentRunning = false;
        System.out.println("Run experiment command initilized");
    }

    @Override
    public void execute() {
        //System.out.println("Experiment Running: " + experimentRunning);
        //System.out.println("Currentspeed " + spiningMotor.getSpeed());
        //System.out.println("Desired speed" + desiredSpeed);
        if (experimentRunning == false && spiningMotor.getSpeed() >= desiredSpeed) {
            experimentRunning = true;
            System.out.println("LETS RUN THIS EXPERIMENT");
        }
        if (experimentRunning == true && cycles > 0) {
            stopwatch.start();
            //System.out.println("stopwatch set");
            tofActivated = tofSensor.isCoralDetected();
            if (tofActivated && !previousTofActivation) {
                dataValues[cyclesRun] = stopwatch.get();
                cycles -= 1;
                cyclesRun += 1;
                System.out.println("Num cycles: " + ((Integer) cycles).toString());
            }
            previousTofActivation = tofActivated;
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

    @Override
    public boolean isFinished() {
        return cycles == 0;
    }

    @Override
    public void end(boolean interrupted) {
        // prints out final data
        spiningMotor.motorStop();
        double meanValue = 0;
        double[] varianceData = new double[cyclesRun];
        System.out.println("Here is the Final Data:");
        for (int i = 1; i < cyclesRun; i++) {
            System.out.println(dataValues[i]);
            double x = dataValues[i] - dataValues[i - 1];
            // System.out.println(x);
            meanValue += x;
            varianceData[i - 1] = x;
        }
        System.out.println();
        for (int i = 1; i < cyclesRun; i++) {
            System.out.println(varianceData[i]);
        }
        meanValue = meanValue/cyclesRun;
        System.out.println("The speed of the experiment was: " + constSpeed);
        System.out.println("Num Cycles run " + cyclesRun);
        System.out.println("The mean time between cycles was: " + meanValue);
        System.out.println("The variance was: " + calculateVariance(varianceData));
        isDone = true;

    }
}
