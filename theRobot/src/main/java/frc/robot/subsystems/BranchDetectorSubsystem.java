// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: BranchDetectorSubsystem.java
// Intent: Forms a subsystem to detect branches using ToFs.
// ************************************************************

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ToFDetector;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
// Network Tables
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;

/**
 * The BranchDetectorSubsystem class is responsible for detecting branches using ToFs
 */
public class BranchDetectorSubsystem extends SubsystemBase{
    
    ToFDetector tofLeft = new ToFDetector(Constants.tofLeftCanID, Constants.branchDetectionThresholdInches);
    ToFDetector tofRight = new ToFDetector(Constants.tofRightCanID, Constants.branchDetectionThresholdInches);


    // Network Tables 
    NetworkTableInstance toFPlotting = NetworkTableInstance.getDefault();
    NetworkTable rightTable = toFPlotting.getTable("datatable");
    NetworkTable leftTable = toFPlotting.getTable("datatable");

    // get a topic from a NetworkTableInstance
    // the topic name in this case is the full name
    //DoubleTopic dblTopic = toFPlotting.getDoubleTopic("/datatable/X");

    // get a topic from a NetworkTable
    // the topic name in this case is the name within the table;
    // this line and the one above reference the same topic
    DoubleTopic rightTopic = rightTable.getDoubleTopic("X");
    DoubleTopic leftTopic = leftTable.getDoubleTopic("X");

    // get a type-specific topic from a generic Topic
    //Topic genericTopic = toFPlotting.getTopic("/datatable/X");
    //DoubleTopic dblTopic = new DoubleTopic(genericTopic);
    final DoubleEntry rightEntry;
    final DoubleEntry leftEntry;


    /**
     * Constructs a BranchDetectorSubsystem object
     */
    public BranchDetectorSubsystem(){

        // start publishing; the return value must be retained (in this case, via
        // an instance variable)
        rightEntry = rightTopic.getEntry(0.0);
        leftEntry = leftTopic.getEntry(0.0);

        // publish options may be specified using PubSubOption
        //dblPub = dblTopic.publish(PubSubOption.keepDuplicates(true));

        // publishEx provides additional options such as setting initial
        // properties and using a custom type string. Using a custom type string for
        // types other than raw and string is not recommended. The properties string
        // must be a JSON map.
        //dblPub = dblTopic.publishEx("double", "{\"myprop\": 5}");

    }
    
    /**
     * A method that returns true if either ToF detects something in range
     * @return - if somehthing detected by either ToF is in range
     */
    public boolean isBranchDetected(){
        boolean leftDetected = InstalledHardware.BranchTofLeft && tofLeft.isDetected();
        boolean rightDetected = InstalledHardware.BranchTofRight && tofRight.isDetected();

        return leftDetected || rightDetected;

    }

    /**
     * A method to run during periodic for the branch detection subsystem
     * it displays TOF output onto advantageScope
     */
    @Override
    public void periodic() {
        // publish a value with a specific timestamp; NetworkTablesJNI.now() can
        // be used to get the current time. On the roboRIO, this is the same as
        // the FPGA timestamp (e.g. RobotController.getFPGATime())
        rightEntry.get(tofRight.getRangeInches());
        leftEntry.get(tofLeft.getRangeInches());

        rightEntry.set(tofRight.getRangeInches());
        leftEntry.set(tofLeft.getRangeInches());
    }

    // often not required in robot code, unless this class doesn't exist for
    // the lifetime of the entire robot program, in which case close() needs to be
    // called to stop publishing
    // NOT USED
    public void close() {
        rightEntry.close();
        leftEntry.close();
    }
}
