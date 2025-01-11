package frc.robot.common;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class reefTofSensor {
    private static final double noteDetectionThreshold = 8.0;
    private TimeOfFlight tofSensor;
    private int canID;
    private String displayName;

    public reefTofSensor(int canID){
        tofSensor = new TimeOfFlight(canID);
        this.canID = canID;
        this.displayName = "TOF ID" + this.canID;

        tofSensor.setRangingMode(RangingMode.Short, 20);
        System.out.println("DONE CONFIG of TOF SENSOR at CanID" + canID);
    }

    public void blinkSensor(){
        tofSensor.identifySensor();
    }

    public String getDisplayName(){
        return displayName;
    }

    public double getRangeInches(){
        return Units.metersToInches(tofSensor.getRange()/1000);
    }

    public final double getRangeSigme(){
        return tofSensor.getRangeSigma();
    }

    public boolean isCoralDetected(){
        double currentRangeInches = this.getRangeInches();
        if(this.isRangeValid() && (currentRangeInches < noteDetectionThreshold)){
            return true;
        }
        return false;
    }

    public boolean isRangeValid(){
        return tofSensor.isRangeValid();
    }

    public void publishTelemetery(){
        SmartDashboard.putNumber(displayName + " Range Inches" , this.getRangeInches());
        SmartDashboard.putBoolean(displayName + " Note Detected", this.isCoralDetected());
        SmartDashboard.putBoolean(displayName + " Range Is Valid", this.isRangeValid());
        SmartDashboard.putString(displayName + " TOF Status", this.tofSensor.getStatus().toString());
      } 
    
}



