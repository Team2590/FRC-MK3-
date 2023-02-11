package frc.auto.trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.Drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;


public class NemesisPath {
    private final Timer timer = new Timer();
    private boolean isStarted;
    private boolean isFinished;
    private List<PathPlannerTrajectory> pathGroup;
    private Trajectory path;
    private PathPlannerTrajectory ontheFly;
    
    public NemesisPath(TrajectoryConfig config, Pose2d initialPose, Pose2d finalPose2d, Translation2d[] waypoints){
        isFinished = false;
        isStarted = false;
        path = TrajectoryGenerator.generateTrajectory(initialPose, List.of(waypoints), finalPose2d, config); 
    }
    public NemesisPath(String pathJSON, double maxVelocity, double maxAcceleration){       
        isFinished = false;
        isStarted = false;
        
            pathGroup = PathPlanner.loadPathGroup(pathJSON, new PathConstraints(maxVelocity, maxAcceleration));
            System.out.println("PATH INITIALIZED");
        
     
    }
    public NemesisPath(PathPoint initial,PathPoint ending, double maxVelocity, double maxAcceleration){
        isFinished=false;
        isStarted=false;
        ontheFly=PathPlanner.generatePath(new PathConstraints(maxVelocity, maxAcceleration), initial, ending);
        pathGroup.add(0,ontheFly);
    }//on the fly 
    public void runPath(Drivetrain drivetrain, int pathNum){
        if(!isStarted){
        // start it
            SmartDashboard.putBoolean("running_path", true);
            
            SmartDashboard.putNumber("path_parts", pathGroup.size());
            drivetrain.resetEncoder();
            drivetrain.resetOdometry(pathGroup.get(0).getInitialHolonomicPose(), drivetrain.getSwervePositions());
            timer.reset();
            timer.start(); 
            isStarted = true;
        } 
        else if(isStarted) {
            // follow the path. Send command to drivetrain
            double currentTime = timer.get();
            SmartDashboard.putNumber("Time:", currentTime);
            SmartDashboard.putNumber("Sample X:", path.sample(currentTime).poseMeters.getX());
            SmartDashboard.putNumber("Sample Y:", path.sample(currentTime).poseMeters.getY());
            drivetrain.followPath(pathGroup.get(pathNum).sample(currentTime));
                

                
            
            
             
        }
    }
    public boolean getIsStarted(){
        return isStarted;
    }
    public boolean getIsFinished(int pathNum){
        isFinished = timer.get() > pathGroup.get(pathNum).getTotalTimeSeconds();
        return isFinished;
    }
    public double getTime(){
        return timer.get();
    }
    public List<EventMarker> getEventMarkers(int pathNum){
        return pathGroup.get(pathNum).getMarkers();
    }
    public State getEventState(int time, int pathNum){
        return pathGroup.get(pathNum).getStates().get(time);
    }
    public void setWait(long duration){
        try{
            path.wait(duration);
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}
