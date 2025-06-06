// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotPosesForReef.java
// Intent: Provides methods to manage and retrieve the positions and orientations
//         of a robot based on predefined tag IDs for both red and blue alliances.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.Constants;
import edu.wpi.first.math.util.Units;

/**
 * The RobotPosesForReef class provides methods to manage and retrieve the
 * positions and orientations
 * of a robot based on predefined tag IDs for both red and blue alliances. It
 * also calculates the
 * reef center coordinates and provides a method to get the robot's pose with an
 * offset from the tag's position.
 */
public class RobotPosesForReef {
    private static HashMap<Double, Pose2d> redPoses = new HashMap<>();
    static {
        redPoses.put(6.0,
                new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-60)));
        redPoses.put(7.0,
                new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0)));
        redPoses.put(8.0,
                new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60)));
        redPoses.put(9.0,
                new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120)));
        redPoses.put(10.0,
                new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180)));
        redPoses.put(11.0,
                new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-120)));
    }

    private static HashMap<Double, Pose2d> bluePoses = new HashMap<>();
    static {
        bluePoses.put(17.0,
                new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(60)));
        bluePoses.put(18.0,
                new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0)));
        bluePoses.put(19.0,
                new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(-60)));
        bluePoses.put(20.0,
                new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.23), Rotation2d.fromDegrees(-120)));
        bluePoses.put(21.0,
                new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180)));
        bluePoses.put(22.0,
                new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(120)));
    }

    // The coordinates of the reef center is the average of the two opposing reef
    // tags
    private static Translation2d redReefCenter = new Translation2d(
            (redPoses.get(7.0).getX() + redPoses.get(10.0).getX()) / 2.0, redPoses.get(7.0).getY());
    private static Translation2d blueReefCenter = new Translation2d(
            (bluePoses.get(18.0).getX() + bluePoses.get(21.0).getX()) / 2.0, bluePoses.get(18.0).getY());

    /**
     * Returns a Pose2d object representing the position and orientation of a robot
     * based on a given tag ID with an offset from the tag's position.
     *
     * @param tagID The ID of the tag to get the pose from. If the tag ID is -1, a
     *              default Pose2d is returned.
     * @return A Pose2d object representing the position and orientation of the
     *         robot.
     *         If the tag ID is not found in either redPoses or bluePoses, a default
     *         Pose2d is returned.
     */
    public static Pose2d getPoseFromTagIDWithOffset(double tagID) {
        Pose2d tagPose;
        Translation2d tagCoordinate;
        boolean isRed;

        if (redPoses.containsKey(tagID)) {
            tagPose = redPoses.get(tagID);
            isRed = true;
        } else if (bluePoses.containsKey(tagID)) {
            tagPose = bluePoses.get(tagID);
            isRed = false;
        } else {
            return new Pose2d();
        }

        tagCoordinate = tagPose.getTranslation();
        Translation2d directionalVec;

        // the direction of the offset vector is the same as the direction from the reef
        // center to the tag.
        if (isRed) {
            directionalVec = tagCoordinate.minus(redReefCenter);
        } else {
            directionalVec = tagCoordinate.minus(blueReefCenter);
        }

        // scale the directional vector by its own magnitude and then divide by the
        // actual offset distance we desire
        Translation2d offsetVector = directionalVec.div(directionalVec.getNorm())
                .times(Constants.alignDistanceFromReefMeters);

        // add the offset vector to the tag to get the destination coordinates
        Translation2d destination = tagCoordinate.plus(offsetVector);

        // we make an assumption that when we see a red tag we are on the red alliance
        // to simplify code
        if (isRed) {
            return new Pose2d(destination, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
        } else {
            return new Pose2d(destination, tagPose.getRotation());
        }
    }

    public static boolean isReefTag(double id) {
        return (id <= 11 && id >= 6) || (id <= 22 && id >= 17);
    }
}
