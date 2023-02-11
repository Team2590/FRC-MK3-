package frc.auto.routines;

import com.pathplanner.lib.PathPoint;

import frc.auto.AutoManager;
import frc.auto.AutoRoutine;
import frc.auto.trajectory.NemesisPath;
import frc.auto.trajectory.PathContainer;
import frc.robot.Robot;
import frc.settings.DrivetrainSettings;
import frc.settings.FieldSettings;
import frc.subsystems.BarIndexer;
import frc.subsystems.Drivetrain;
import frc.subsystems.Suction;
import frc.util.Limelight;

/**
 * Drive in an S path and Spin routine 
 * @author Abhik Ray
 */
public class automatic_choice extends AutoRoutine implements DrivetrainSettings, FieldSettings{
    // NemesisPath forwardS = PathContainer.moveForward;
    // NemesisPath reverseS = PathContainer.moveReverse;
    // NemesisPath spin = PathContainer.spinInPlace;
    NemesisPath CommunityPlacement=PathContainer.automatic_choice;

    Drivetrain driveT;
    Suction suction;
    Limelight lime;
    double x_offset;
    BarIndexer bar;
    private double time;
    private enum States {
         ALIGN, PLACE, FIRST_MOVE, PICKUP, SECOND_MOVE, BALANCE, CHOICE, PLACE2, THIRD_MOVE, END 
    }
    
    public States autoState;
    public automatic_choice(){
        autoState = States.ALIGN;
    }
    
    private int pathPart;
    private double levelError;
    private int tagID;
    private PathPoint targetPoint;
    private NemesisPath ontheFly;

    @Override
    public String getKey() {
        // TODO Auto-generated method stub
        return "place_balance";
    }

    @Override
    public void initialize() {
        System.out.println("INITALIAIRD");
        // TODO Auto-generated method stub
        driveT = Robot.getDrivetrainInstance();
        suction=Robot.getSuctionInstance();
        lime=new Limelight();
        suction.succToggle();
        pathPart=0;
        bar=Robot.getIndexerInstance();
        
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        time=CommunityPlacement.getTime();
        lime.update();
        driveT.update();
        bar.update();
        suction.update();
        driveT.setVisionMatrices();
        if (lime.hasTarget()){
            driveT.updateVision(lime.getVisionLocalization());




        }

        
        

        double x_offset=lime.getDistance();

        levelError=driveT.getLevelEror();
        switch(autoState){
             
                 
                
                
            case ALIGN:
                 driveT.aligning(x_offset);
                 autoState=States.PLACE;

                 
                 
                
                
                break;
            case PLACE:
                //elevator.UP-Position
                 suction.stop();
                 autoState=States.FIRST_MOVE;
                
                
                break;
            case PLACE2:
                //elevator.UP-Position
                suction.stop();
                autoState=States.THIRD_MOVE;
            case THIRD_MOVE:
                ontheFly=driveT.generatePath(balancePP);
                ontheFly.runPath(driveT, 0);
                if (ontheFly.getIsFinished(0)){
                        
    
                    autoState=States.BALANCE;
                }


                break;

            case FIRST_MOVE://pp=0
                CommunityPlacement.runPath(driveT,pathPart);
                if (CommunityPlacement.getIsFinished(pathPart)){
                    pathPart=1;

                 autoState=States.PICKUP;
             }
                 
                
            case BALANCE:
                driveT.autoLevel();
                if(levelError<.02){
                    autoState=States.END;

                }
                
                break;
            case CHOICE:

                if (lime.hasTarget()){
                    tagID=lime.getAprilTagID();
                    targetPoint=idPoses.get(tagID);
                    ontheFly=driveT.generatePath(targetPoint);
                    ontheFly.runPath(driveT, 0);
                    if (ontheFly.getIsFinished(0)){
                        
    
                     autoState=States.PLACE2;
                 }


                    


                }
                
                
                break;
            

                
                
           

            case PICKUP:
                bar.toggleIndexer();
                suction.succToggle();
                autoState=States.SECOND_MOVE;



                break;
            case SECOND_MOVE://pp=1
                CommunityPlacement.runPath(driveT, pathPart);
                if (CommunityPlacement.getIsFinished(pathPart)){
                    pathPart=2;

                 autoState=States.CHOICE;
             }


                
                break;
            
            case END:
                driveT.stop();
                System.out.println("done yippee ");
            
            default:
                break;
             


        }
        // System.out.println("running auto");
        
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        driveT.stop();
        
    }

}
