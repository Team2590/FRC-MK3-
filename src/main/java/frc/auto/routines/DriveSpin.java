package frc.auto.routines;

import frc.auto.AutoManager;
import frc.auto.AutoRoutine;
import frc.auto.trajectory.NemesisPath;
import frc.auto.trajectory.PathContainer;
import frc.robot.Robot;
import frc.subsystems.Drivetrain;
import frc.subsystems.DrivetrainSettings;

/**
 * Drive in an S path and Spin routine 
 * @author Abhik Ray
 */
public class DriveSpin extends AutoRoutine implements DrivetrainSettings{
    NemesisPath forwardS = PathContainer.moveForward;
    NemesisPath reverseS = PathContainer.moveReverse;
    NemesisPath spin = PathContainer.spinInPlace;
    Drivetrain driveT;
    private enum States {
        S_MOVEMENT, SPIN, RETURN, CIRCLE, END 
    }
    public States autoState;
    public DriveSpin(){
        autoState = States.S_MOVEMENT;
    }

    @Override
    public String getKey() {
        // TODO Auto-generated method stub
        return "DriveSpin";
    }

    @Override
    public void initialize() {
        System.out.println("INITALIAIRD");
        // TODO Auto-generated method stub
        driveT = Robot.getDrivetrainInstance();
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        switch(autoState){
            case S_MOVEMENT:
                forwardS.runPath(driveT);
                if(forwardS.getIsFinished()){
                    autoState = States.SPIN;
                }
                break;
            case SPIN:
                spin.runPath(driveT);
                if(spin.getIsFinished()){
                    autoState = States.RETURN;
                }
                break;
            case RETURN:
                reverseS.runPath(driveT);
                if(reverseS.getIsFinished()){
                    autoState = States.END;
                }
                break;
            case CIRCLE:
                break;
            case END:
                System.out.println("done!");
                break;


        }
        // System.out.println("running auto");
        
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        
    }

}
