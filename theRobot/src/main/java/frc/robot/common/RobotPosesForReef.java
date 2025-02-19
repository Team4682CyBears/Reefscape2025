package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.Constants;
import edu.wpi.first.math.util.Units;

public class RobotPosesForReef {
    private static Pose2d tag6 = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-60));
    private static Pose2d tag7 = new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0));
    private static Pose2d tag8 = new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60));
    private static Pose2d tag9 = new Pose2d( Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120));
    private static Pose2d tag10 = new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180));
    private static Pose2d tag11 = new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(-120));
    private static Pose2d tag17 = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(60));
    private static Pose2d tag18 = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0));
    private static Pose2d tag19 = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(-60));
    private static Pose2d tag20 = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.23), Rotation2d.fromDegrees(-120));
    private static Pose2d tag21 = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180));
    private static Pose2d tag22 = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(120));
    // TODO rather than storing this in an array, should store it in a hashmap
    // If you stored it in two hash maps (one for red and one for blue), 
    // you could use the presence of it to also know which side you are on without having to do the tag comparison below
    private static Pose2d[] listOfPoses = {tag6, tag7, tag8, tag9, tag10, tag11, 
                            tag17, tag18, tag19, tag20, tag21, tag22};

    // The coordinates of the reef center is the average of the two opposing reef tags
    private static Translation2d redReefCenter = new Translation2d((tag7.getX()+tag10.getX())/2.0, tag7.getY());
    private static Translation2d blueReefCenter = new Translation2d((tag18.getX()+tag21.getX())/2.0, tag18.getY());

    public static Pose2d getPoseFromTagID(double tagID) {
        if (tagID == -1){
            return new Pose2d();
        }
        else if(tagID <= 11 && tagID >= 6){
            return listOfPoses[(int)(tagID - 6.0)];
        }
        else if (tagID <= 22 && tagID >= 17){
            return listOfPoses[(int)(tagID - 11.0)];
        }
        else{
            return new Pose2d();
        }
    }

    public static Pose2d getPoseFromTagIDWithOffset(double tagID) {
        Pose2d tagPose;
        Translation2d tagCoordinate;
        boolean isRed;

        //-1 is the id when we dont see a tag
        if (tagID == -1){
            return new Pose2d();
        }
        else if(tagID <= 11 && tagID >= 6){
            tagPose = listOfPoses[(int)(tagID - 6.0)];
            isRed = true;

        }
        else if (tagID <= 22 && tagID >= 17){
            tagPose = listOfPoses[(int)(tagID - 11.0)];
            isRed = false;
        }
        else{
            return new Pose2d();
        }

        System.out.println("Getting PoseFromTagIDWithOffset for tag " + tagID);

        System.out.println("Tag Pose " + tagPose);
        System.out.println("isRed " + isRed);

        tagCoordinate = tagPose.getTranslation();
        Translation2d directionalVec;

        // the direction of the offset vector is the same as the direction from the reef center to the tag. 
        if(isRed) {
            directionalVec = tagCoordinate.minus(redReefCenter);
            System.out.println("Using Red Reef center " + redReefCenter);
        }
        else {
            directionalVec = tagCoordinate.minus(blueReefCenter);
            System.out.println("Using Blue Reef center " + blueReefCenter);
        }

        System.out.println("directionalVec" + directionalVec);

        // scale the directional vector by its own magnitude and then divide by the actual offset distance we desire
        Translation2d offsetVector = directionalVec.div(directionalVec.getNorm()).times(Constants.alignDistanceFromReefMeters);

        // add the offset vector to the tag to get the destination coordinates
        Translation2d destination = tagCoordinate.plus(offsetVector);
        System.out.println("Destination " + destination);


        // TODO change this function to use the drivetrain's operator perspective variable. 
        if(isRed){   
            return new Pose2d(destination, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
        }
        else {
            return new Pose2d(destination, tagPose.getRotation());
        }
    }
}
