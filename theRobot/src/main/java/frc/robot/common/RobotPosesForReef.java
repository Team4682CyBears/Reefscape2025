package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.Constants;

public class RobotPosesForReef {
    private static Pose2d tag6 = new Pose2d(4.700446000000001, -0.7196820000000002, Rotation2d.fromDegrees(-60));
    private static Pose2d tag7 = new Pose2d(5.116498, -0.00009999999999976694, Rotation2d.fromDegrees(0));
    private static Pose2d tag8 = new Pose2d(4.700446000000001, 0.7194820000000002, Rotation2d.fromDegrees(60));
    private static Pose2d tag9 = new Pose2d( 3.869358, 0.7194820000000002, Rotation2d.fromDegrees(120));
    private static Pose2d tag10 = new Pose2d(3.4533059999999995, -0.00009999999999976694, Rotation2d.fromDegrees(180));
    private static Pose2d tag11 = new Pose2d(3.869358, -0.7196820000000002, Rotation2d.fromDegrees(-120));
    private static Pose2d tag17 = new Pose2d(-4.700094, -0.7196820000000002, Rotation2d.fromDegrees(60));
    private static Pose2d tag18 = new Pose2d(-5.116399999999999,  -0.00009999999999976694, Rotation2d.fromDegrees(0));
    private static Pose2d tag19 = new Pose2d(-4.700094, 0.7194820000000002, Rotation2d.fromDegrees(-60));
    private static Pose2d tag20 = new Pose2d(-3.8692599999999997, 0.7194820000000002, Rotation2d.fromDegrees(-120));
    private static Pose2d tag21 = new Pose2d(-3.452953999999999, -0.00009999999999976694, Rotation2d.fromDegrees(180));
    private static Pose2d tag22 = new Pose2d(-3.8692599999999997, -0.7196820000000002, Rotation2d.fromDegrees(120));
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

        tagCoordinate = tagPose.getTranslation();
        Translation2d directionalVec;

        // the direction of the offset vector is the same as the direction from the reef center to the tag. 
        if(isRed) {
            directionalVec = tagCoordinate.minus(redReefCenter);
        }
        else {
            directionalVec = tagCoordinate.minus(blueReefCenter);
        }
        // scale the directional vector by its own magnitude and then divide by the actual offset distance we desire
        Translation2d offsetVector = directionalVec.div(directionalVec.getNorm()).times(Constants.alignDistanceFromReef);
        // add the offset vector to the tag to get the destination coordinates
        Translation2d destination = tagCoordinate.plus(offsetVector);
        if(isRed){   
            return new Pose2d(destination, tagPose.getRotation().plus(new Rotation2d(Math.PI)));
        }
        else {
            return new Pose2d(destination, tagPose.getRotation());
        }
    }
}
