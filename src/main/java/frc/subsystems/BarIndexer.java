package frc.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;
import frc.settings.BarIndexerSettings;

public class BarIndexer implements RobotMap, Subsystem, BarIndexerSettings {
    
    private Solenoid barSol;

    private static BarIndexer indexer = null;
    private enum States {
        STOPPED, ON
    }
    private States indexState;
    private boolean indexSolStatus;

    public BarIndexer(PowerDistribution pdp){
        indexState = States.STOPPED;
        indexSolStatus = false;
        barSol = new Solenoid(PneumaticsModuleType.CTREPCM, INDEXER_SOLENOID);
    }
    public static BarIndexer getIndexerInstance(PowerDistribution pdp) {
        if (indexer == null) {
            indexer = new BarIndexer(pdp);
        }
        return indexer;
    }
    public void update(){
        switch(indexState){
            case ON:
                SmartDashboard.putBoolean("Index status", indexSolStatus);
                barSol.set(indexSolStatus);
                break;
            case STOPPED:
                barSol.set(false);
                break;
        }
    }
    public void toggleIndexer(){
        indexState=States.ON;
        indexSolStatus = !indexSolStatus;
    }

}