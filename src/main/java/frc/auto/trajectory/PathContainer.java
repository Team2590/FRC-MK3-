package frc.auto.trajectory;

import java.lang.reflect.Method;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import frc.auto.routines.CommunityPlacement;
import frc.auto.routines.automatic_choice;
import frc.settings.DrivetrainSettings;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.subsystems.Suction;



public class PathContainer implements DrivetrainSettings{

    // static String RoboRios_curveJSOn="paths/output/AyanPath.wpilib.json";  
      
    


    // trajectory config should be the same for all paths
    static TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(swerveKinematics);
        // Apply the voltage constraint
        // .addConstraint(autoVoltageConstraint);

    static TrajectoryConfig REVERSED_CONFIG = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(swerveKinematics)
        .setReversed(true);
        // Apply the voltage constraint
        // .addConstraint(autoVoltageConstraint);
        private final static Pose2d startingPosition = new Pose2d(0,0, new Rotation2d(0));
        private final static Pose2d endingPosition = new Pose2d(0,-3.175, new Rotation2d(0));
        // Below will instantiate new paths 
        // static Translation2d[] moveForwardWaypoints = {new Translation2d(0,-1.5875)};
        // public static NemesisPath moveForward = new NemesisPath(TRAJECTORY_CONFIG, 
        //     startingPosition, 
        //     endingPosition, 
        //     moveForwardWaypoints
        // );

        // static Translation2d[] spinWaypoints = { new Translation2d(0.0,-5) };
        // public static NemesisPath spinInPlace = new NemesisPath(TRAJECTORY_CONFIG,  
        //     endingPosition, 
        //     new Pose2d(0.0,-5, new Rotation2d(1*Math.PI)), 
        //     spinWaypoints
        // );
        // static Translation2d[] reverseWaypoints = {new Translation2d(0,-1.5875)};
        // public static NemesisPath moveReverse = new NemesisPath(REVERSED_CONFIG, 
        //     new Pose2d(0,-5, new Rotation2d(2*Math.PI)), 
        //     startingPosition, 
        //     reverseWaypoints
        // );

        // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
        // public static NemesisPath place_balance=new NemesisPath("place_balance",MAX_VELOCITY,MAX_ACCELERATION);
        // public static NemesisPath CommunityPlacement= new NemesisPath("CommunityPlacement",MAX_VELOCITY, MAX_ACCELERATION);
        // public static NemesisPath automatic_choice=new NemesisPath("automatic_choice",MAX_VELOCITY, MAX_ACCELERATION);
        public static ArrayList<NemesisPath> getCommunityPaths(){
            ArrayList<NemesisPath> communityPaths = new ArrayList<NemesisPath>();
            communityPaths.add(new NemesisPath("CommunityPlacement", MAX_VELOCITY, MAX_ACCELERATION,0));       
            communityPaths.add(new NemesisPath("CommunityPlacement", MAX_VELOCITY, MAX_ACCELERATION, 1));
            communityPaths.add(new NemesisPath("CommunityPlacement", MAX_VELOCITY, MAX_ACCELERATION, 2));
            return communityPaths;
        }
        public static ArrayList<NemesisPath> getAutomatedPaths(){
            ArrayList<NemesisPath> automatedPaths = new ArrayList<NemesisPath>();
            automatedPaths.add(new NemesisPath("auto_choice", MAX_VELOCITY, MAX_ACCELERATION,0));       
            automatedPaths.add(new NemesisPath("auto_choice", MAX_VELOCITY, MAX_ACCELERATION, 1));
            return automatedPaths;
        }
}         


