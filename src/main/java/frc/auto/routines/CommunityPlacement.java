package frc.auto.routines;

import frc.auto.AutoManager;
import frc.auto.AutoRoutine;
import frc.auto.trajectory.NemesisPath;
import frc.auto.trajectory.PathContainer;
import frc.robot.Robot;
import frc.settings.DrivetrainSettings;
import frc.subsystems.BarIndexer;
import frc.subsystems.Drivetrain;
import frc.subsystems.Suction;
import frc.util.Limelight;

/**
 * Drive in an S path and Spin routine 
 * @author Abhik Ray
 */
public class CommunityPlacement extends AutoRoutine implements DrivetrainSettings{
    // NemesisPath forwardS = PathContainer.moveForward;
    // NemesisPath reverseS = PathContainer.moveReverse;
    // NemesisPath spin = PathContainer.spinInPlace;
    NemesisPath CommunityPlacement=PathContainer.CommunityPlacement;

    Drivetrain driveT;
    Suction suction;
    // Limelight lime;
    double x_offset;
    BarIndexer bar;
    private double time;
    private enum States {
         ALIGN, PLACE, FIRST_MOVE, PICKUP, SECOND_MOVE, BALANCE, PLACE2, THIRD_MOVE, END 
    }
    
    public States autoState;
    public CommunityPlacement(){
        autoState = States.FIRST_MOVE;
    }
    
    private int pathPart;
    private double levelError;

    @Override
    public String getKey() {
        // TODO Auto-generated method stub
        return "Community_Placement";
    }

    @Override
    public void initialize() {
        System.out.println("INITALIAIRD");
        // TODO Auto-generated method stub
        driveT = Robot.getDrivetrainInstance();
        // suction=Robot.getSuctionInstance();
        // lime=new Limelight("limelight_low");
       // suction.succToggle();
        pathPart=0;
        //bar=Robot.getIndexerInstance();
        
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        time=CommunityPlacement.getTime();
        // lime.update();
        driveT.update();
       // bar.update();
       // suction.update();

        
        

        // double x_offset=lime.getDistance();

        levelError=driveT.getLevelEror();
        switch(autoState){
             
                 
                
                
            case ALIGN:
                 driveT.aligning(x_offset);
                 autoState=States.PLACE;

                 
                 
                
                
                break;
            case PLACE:
                //elevator.UP-Position
                // suction.stop();
                 autoState=States.FIRST_MOVE;
                
                
                break;
            case PLACE2:
                //elevator.UP-Position'
                driveT.aligning(x_offset);
               // suction.stop();
                autoState=States.THIRD_MOVE;

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
            

                
                
            case THIRD_MOVE://PP=2
            CommunityPlacement.runPath(driveT,pathPart);
            if (CommunityPlacement.getIsFinished(pathPart)){
                

             autoState=States.BALANCE;
         }

                break;
            case PICKUP:
                //bar.toggleIndexer();
                //suction.succToggle();
                autoState=States.SECOND_MOVE;



                break;
            case SECOND_MOVE://pp=1
                CommunityPlacement.runPath(driveT, pathPart);
                if (CommunityPlacement.getIsFinished(pathPart)){
                    pathPart=2;

                 autoState=States.PLACE2;
             }


                
                break;
            
            case END:
                driveT.stop();
                System.out.println("done yippee ");
             


        }
        // System.out.println("running auto");
        
    }

    @Override
    public void end() {
        // TODO Auto-generated method stub
        driveT.stop();
        
    }

}
