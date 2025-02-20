// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: DrivetrainConfig.java
// Intent: Forms a type to hold drivetrain config
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

/**
 * Forms a type to hold drivetrian config
 */
public class DrivetrainSwerveConfig {
    /**
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    private double trackwidthMeters;
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    private double wheelbaseMeters;

    public DrivetrainSwerveConfig(
            double trackWidthMeters,
            double wheelbaseMeters) {
        this.trackwidthMeters = trackWidthMeters;
        this.wheelbaseMeters = wheelbaseMeters;
    }

    public double getTrackwidthMeters() {
        return trackwidthMeters;
    }

    public double getWheelbaseMeters() {
        return wheelbaseMeters;
    }

    public void setTrackwidthMeters(double trackwidthMeters) {
        this.trackwidthMeters = trackwidthMeters;
    }

    public void setWheelbaseMeters(double wheelbaseMeters) {
        this.wheelbaseMeters = wheelbaseMeters;
    }

}
