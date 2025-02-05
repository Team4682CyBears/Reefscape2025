package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.control.Constants;

public class RobotPosesForReef {
    private static Pose2d tag6 = new Pose2d(4.700446000000001, -0.7196820000000002, new Rotation2d(60));
    private static Pose2d tag7 = new Pose2d(5.116498, -0.00009999999999976694, new Rotation2d(0));
    private static Pose2d tag8 = new Pose2d(4.700446000000001, 0.7194820000000002, new Rotation2d(-60));
    private static Pose2d tag9 = new Pose2d( 3.869358, 0.7194820000000002, new Rotation2d(-120));
    private static Pose2d tag10 = new Pose2d(3.4533059999999995, -0.00009999999999976694, new Rotation2d(180));
    private static Pose2d tag11 = new Pose2d(3.869358, -0.7196820000000002, new Rotation2d(120));
    private static Pose2d tag17 = new Pose2d(-4.700094, -0.7196820000000002, new Rotation2d(-60));
    private static Pose2d tag18 = new Pose2d(-5.116399999999999,  -0.00009999999999976694, new Rotation2d(0));
    private static Pose2d tag19 = new Pose2d(-4.700094, 0.7194820000000002, new Rotation2d(60));
    private static Pose2d tag20 = new Pose2d(-3.8692599999999997, 0.7194820000000002, new Rotation2d(120));
    private static Pose2d tag21 = new Pose2d(-3.452953999999999, -0.00009999999999976694, new Rotation2d(180));
    private static Pose2d tag22 = new Pose2d(-3.8692599999999997, -0.7196820000000002, new Rotation2d(-120));
    private static Pose2d[] listOfPoses = {tag6, tag7, tag8, tag9, tag10, tag11, 
                            tag17, tag18, tag19, tag20, tag21, tag22};

    private static Translation2d redReefCenter = new Translation2d(tag7.getX()-tag10.getX(), tag7.getY());
    private static Translation2d blueReefCenter = new Translation2d(tag18.getX()-tag21.getX(), tag18.getY());

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
        System.out.println("Is Red: " + isRed);

        tagCoordinate = tagPose.getTranslation();
        Translation2d directionalVec;
        System.out.println("Pose translation: " + tagCoordinate);

        if(isRed) {
            directionalVec = tagCoordinate.minus(redReefCenter);
        }
        else {
            directionalVec = tagCoordinate.minus(blueReefCenter);
        }
        Translation2d offsetVector = directionalVec.div(directionalVec.getNorm()).times(Constants.alignDistanceFromReef);
        Translation2d destination = tagCoordinate.plus(offsetVector);   

        return new Pose2d(destination, tagPose.getRotation());
    }
}
