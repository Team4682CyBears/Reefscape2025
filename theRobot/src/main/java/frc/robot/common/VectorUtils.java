// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: VectorUtils.java
// Intent: Forms a class of utilities for vector operations
// ************************************************************

package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A collection of methods for vector computations
 */
public class VectorUtils{

    /**
     * Translates the x,y portion of the pose. Leaves the rotation unchanged.
     * Use instead of Pose2d.add, which does strange things when the rotation is non-zero.
     * @param pose
     * @param translation
     * @return new pose
     */
    public static Pose2d translatePose(Pose2d pose, Translation2d translation){
        return new Pose2d(pose.getTranslation().plus(translation), pose.getRotation());
    }
}
