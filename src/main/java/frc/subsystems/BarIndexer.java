package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;
import frc.settings.BarIndexerSettings;

public class BarIndexer implements RobotMap, Subsystem, BarIndexerSettings {
    
    private VictorSPX barMotor;
    private double barPower;

    private static BarIndexer indexer = null;
    private enum States {
        STOPPED, ON
    }
    private States indexState;
    private boolean indexSolStatus;
    public BarIndexer(PowerDistribution pdp){
        indexState = States.STOPPED;
        // indexSolStatus = false;
        barMotor = new VictorSPX(BAR_ID);
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
                barMotor.set(ControlMode.PercentOutput,barPower);
                // barSol.set(indexSolStatus);
                break;
            case STOPPED:
                // barSol.set(false);
                break;
        }
    }
    public void setPower(double power){
        indexState=States.ON;
        barPower = power;
        // indexSolStatus = !indexSolStatus;
    }

}