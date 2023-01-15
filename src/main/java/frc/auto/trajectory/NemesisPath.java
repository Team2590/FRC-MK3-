package frc.auto.trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.Drivetrain;

public class NemesisPath {
    private final Timer timer = new Timer();
    private boolean isStarted;
    private boolean isFinished;
    private Trajectory path;
    public NemesisPath(TrajectoryConfig config, Pose2d initialPose, Pose2d finalPose2d, Translation2d[] waypoints){
        isFinished = false;
        isStarted = false;
        path = TrajectoryGenerator.generateTrajectory(initialPose, List.of(waypoints), finalPose2d, config); 
    }
    public NemesisPath(Path pathJSON){       
        isFinished = false;
        isStarted = false;
        try {
            path = TrajectoryUtil.fromPathweaverJson(pathJSON);
            System.out.println("PATH INITIALIZED");
        } catch (IOException ex) {
            DriverStation.reportError("Path not Found" , ex.getStackTrace());
        }
    }
    public void runPath(Drivetrain drivetrain){
        if(!isStarted){
        // start it
            System.out.println("STARTING PATH"+"reset encoders");
            drivetrain.resetEncoder();
            drivetrain.resetOdometry(path.getInitialPose(), drivetrain.getSwervePositions());
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
    public void setWait(long duration){
        try{
            path.wait(duration);
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}
