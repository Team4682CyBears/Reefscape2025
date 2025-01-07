// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ManualInputInterfaces.java
// Intent: Forms a class with trajcetories for testing swerve drives.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.util.ArrayList;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * A class with trajcetories for testing swerve drives.
 */
public class TestTrajectories {
  // TODO - rewrite this with PathPlanner on-the-fly paths and PathPlanner trajectory follower.
    public Trajectory traverseSimpleForward;
    public Trajectory traverseZigZag;
    public Trajectory traverseSimpleLeft;
    public Trajectory traverseTurn270;
    public Trajectory turn90;
    public Trajectory traverseForwardArc;
    public Trajectory traverseBackwardArc;
    public Pose2d traverseBackwardArcStartPosition = new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0.0));

    /**
    * constructs trajcetories for testing swerve drives.
    */
    public TestTrajectories(){
        traverseSimpleForward = buildTraverseSimpleForward();
        traverseZigZag = buildZigZag();
        traverseSimpleLeft = buildTraverseSimpleLeft();
        traverseTurn270 = buildTraverseTurn270();
        turn90 = buildTurn90();
        traverseForwardArc = buildTraverseForwardArc();
        traverseBackwardArc = buildTraverseBackwardArc();
    }

    private PathPlannerPath buildTraverseSimpleForward(){
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        // TODO write something that takes Pose2d list and outputs list of PathPlanner Waypoints
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0)));
    
        PathPlannerPath t = new PathPlannerPath(
          waypoints, 
          new PathConstraints(1, 1, 1, 1, 12, false), // TODO what are the right constraints??
          new IdealStartingState(0.0, new Rotation2d(0.0)), 
          new GoalEndState(0.0, new Rotation2d(0.0))); // TODO - rewrite this with PathPlanner 
        return t;
      }

      private PathPlannerPath buildZigZag(){
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(1.5, 0.5, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(3.0, -0.5, Rotation2d.fromDegrees(0)));

        Trajectory t = new Trajectory(); // TODO - rewrite this with PathPlanner 
        return t;
    }
    
      private PathPlannerPath buildTraverseSimpleLeft(){
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
        waypoints.add(new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0)));
        Trajectory t = new Trajectory(); // TODO - rewrite this with PathPlanner 
        return t;
      }
    
      private PathPlannerPath buildTraverseTurn270(){
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(-90)));
        Trajectory t = new Trajectory(); // TODO - rewrite this with PathPlanner 
        return t;
      }
    
      // Test purely rotational trajectory.  
      private Trajectory buildTurn90(){
        ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90)));
        Trajectory t = new Trajectory(); // TODO - rewrite this with PathPlanner 
        return t;
      }
    
      private Trajectory buildTraverseForwardArc(){
        Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        Pose2d end = this.traverseBackwardArcStartPosition;
    
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(0.5, 0.25));
        interiorWaypoints.add(new Translation2d(1.0, 0.50));
        interiorWaypoints.add(new Translation2d(1.5, 0.25));
        Trajectory t = new Trajectory(); // TODO - rewrite this with PathPlanner 
        return t;
      }
    
      private Trajectory buildTraverseBackwardArc(){
        Pose2d start = this.traverseBackwardArcStartPosition;
        Pose2d end = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(1.5, 0.25));
        interiorWaypoints.add(new Translation2d(1.0, 0.50));
        interiorWaypoints.add(new Translation2d(0.5, 0.25));
        Trajectory t = new Trajectory(); // TODO - rewrite this with PathPlanner 
        return t;
      }
    
    
}
