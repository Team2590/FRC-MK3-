// package frc.auto.routines;

// import frc.auto.AutoManager;
// import frc.auto.AutoRoutine;
// import frc.auto.trajectory.NemesisPath;
// import frc.auto.trajectory.PathContainer;
// import frc.robot.Robot;
// import frc.settings.DrivetrainSettings;
// import frc.subsystems.Drivetrain;
// import frc.subsystems.Suction;
// import frc.util.Limelight;

// /**
//  * Drive in an S path and Spin routine 
//  * @author Abhik Ray
//  */
// public class place_balance extends AutoRoutine implements DrivetrainSettings{
//     // NemesisPath forwardS = PathContainer.moveForward;
//     // NemesisPath reverseS = PathContainer.moveReverse;
//     // NemesisPath spin = PathContainer.spinInPlace;
//     NemesisPath place_balance=PathContainer.place_balance;

//     Drivetrain driveT;
//     Suction suction;
//     Limelight lime;
//     double x_offset;
//     private double time;
//     private enum States {
//         INIT_MOVE, ALIGN, PLACE, SECOND_MOVE, BALANCE, END 
//     }
    
//     public States autoState;
//     public place_balance(){
//         autoState = States.INIT_MOVE;
//     }
    
//     private int pathPart;
//     private double levelError;

//     @Override
//     public String getKey() {
//         // TODO Auto-generated method stub
//         return "place_balance";
//     }

//     @Override
//     public void initialize() {
//         System.out.println("INITALIAIRD");
//         // TODO Auto-generated method stub
//         driveT = Robot.getDrivetrainInstance();
//         suction=Robot.getSuctionInstance();
//         lime=new Limelight();
//         suction.succToggle();
//         pathPart=0;
        
        
//     }

//     @Override
//     public void update() {
//         // TODO Auto-generated method stub
//         time=place_balance.getTime();
//         lime.update();
//         driveT.update();
        
        

//         double x_offset=lime.getX();

//         levelError=driveT.getLevelEror();
//         switch(autoState){
//              case INIT_MOVE:
//                  place_balance.runPath(driveT,pathPart);
//                  if (place_balance.getIsFinished(pathPart)){
//                     pathPart=1;

//                     autoState=States.ALIGN;
//                  }
                 
                
//                 break;
//             case ALIGN:
//                  driveT.aligning(x_offset);
//                  autoState=States.PLACE;

                 
                 
                
                
//                 break;
//             case PLACE:
//                  suction.stop();
//                  autoState=States.SECOND_MOVE;
                
                
//                 break;
//             case SECOND_MOVE:
//                 place_balance.runPath(driveT,pathPart);
//                 if (place_balance.getIsFinished(pathPart)){
//                  pathPart=2;

//                  autoState=States.BALANCE;
//              }
                 
                
//             case BALANCE:
//                 driveT.autoLevel();
//                 if(levelError<.02){
//                     autoState=States.END;

//                 }
                
//                 break;
//             case END:
//                 driveT.stop();
//                 System.out.println("done yippee ");

                
//                 break;
             


//         }
//         // System.out.println("running auto");
        
//     }

//     @Override
//     public void end() {
//         // TODO Auto-generated method stub
//         driveT.stop();
        
//     }

// }
