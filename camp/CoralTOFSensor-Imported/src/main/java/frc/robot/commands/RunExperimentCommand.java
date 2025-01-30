package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ImplementationTOF;
import frc.robot.subsystems.Spinner;

public class RunExperimentCommand extends Command {
    Spinner spiningMotor;
    ImplementationTOF tofSensor;
    double constSpeed;
    double desiredSpeed = 0;
    boolean isDone = false;
    int desiredCycles;
    int cycles;
    int cyclesRun = 0;
    Timer stopwatch = new Timer();
    boolean tofActivated = false;
    boolean previousTofActivation = false;
    double dataValue;
    double[] dataValues;
    boolean experimentRunning = false;

    public RunExperimentCommand(Double constS, Spinner spin, ImplementationTOF tof, int cyc) {
        spiningMotor = spin;
        tofSensor = tof;
        constSpeed = constS;
        desiredCycles = cyc;
        dataValues = new double[cycles];
        addRequirements(spin, tof);
    }

    @Override
    public void initialize() {
        isDone = false;
        cycles = desiredCycles;
        cyclesRun = 0;
        desiredSpeed = constSpeed;
        spiningMotor.spinMotor(desiredSpeed);
        System.out.println("HEY LOOK AT ME OVER HERE THE DESIRED SPEED IS " + desiredSpeed);
        stopwatch.reset();
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
            System.out.println("MOTOR AT SPEED. RUNNING EXPERIMENT for " + cycles + " cycles.");
        }
        if (experimentRunning == true && cycles > 0) {
            stopwatch.start();
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
        if (interrupted){
            System.out.println("THE COMMAND WAS INTERRUPTED!!");
        }
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
