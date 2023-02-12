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
    private PathPlannerTrajectory path;
    private PathPlannerTrajectory ontheFly;
    
    // public NemesisPath(TrajectoryConfig config, Pose2d initialPose, Pose2d finalPose2d, Translation2d[] waypoints){
    //     isFinished = false;
    //     isStarted = false;
    //     path = TrajectoryGenerator.generateTrajectory(initialPose, List.of(waypoints), finalPose2d, config); 
    // }
    public NemesisPath(String pathJSON, double maxVelocity, double maxAcceleration, int part, boolean reverse){
            isFinished = false;
            isStarted = false;
            path=PathPlanner.loadPathGroup(pathJSON,maxVelocity*.5, maxAcceleration*.5,reverse).get(part);
            // pathGroup = PathPlanner.loadPathGroup(pathJSON, new PathConstraints(maxVelocity, maxAcceleration));
            System.out.println("PATH INITIALIZED");
        
     
    }
    public NemesisPath(PathPoint initial,PathPoint ending, double maxVelocity, double maxAcceleration){
        isFinished=false;
        isStarted=false;
        ontheFly=PathPlanner.generatePath(new PathConstraints(maxVelocity, maxAcceleration), initial, ending);
        // pathGroup.add(0,ontheFly);
    }//on the fly 
    public void runPath(Drivetrain drivetrain){
        if(!isStarted){
        // start it
            System.out.println("STARTING PATH"+"reset encoders");
            
           
            drivetrain.resetOdometry(path.getInitialHolonomicPose(), drivetrain.getSwervePositions());
            SmartDashboard.putString("Path Initial Pose", path.getInitialHolonomicPose().toString());

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
            SmartDashboard.putNumber("Sample  velocity", path.sample(currentTime).velocityMetersPerSecond);
            drivetrain.followPath(path.sample(currentTime));
                

                
            
            
             
        }
    }
    public boolean getIsStarted(){
        return isStarted;
    }
    public boolean getIsFinished(){
        isFinished = timer.get() > path.getTotalTimeSeconds();
        return isFinished;
    }
    public double getTime(){
        return timer.get();
    }
    public List<EventMarker> getEventMarkers(){
        return path.getMarkers();
    }
    public State getEventState(int time){
        return path.getStates().get(time);
    }
    public void setWait(long duration){
        try{
            path.wait(duration);
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}
