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
    private VictorSPX barMotorL;
    private VictorSPX barMotorR;
    private VictorSPX lift;
    private double barPower;
    private double liftPower;

    private static BarIndexer indexer = null;
    private enum States {
        STOPPED, ON
    }
    private States indexState;
    private boolean indexSolStatus;
    public BarIndexer(PowerDistribution pdp){
        indexState = States.STOPPED;
        barMotorL = new VictorSPX(BAR_L_ID);
        barMotorR = new VictorSPX(BAR_R_ID);
        lift = new VictorSPX(BLOWER);
        barMotorL.setInverted(true);
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
                move(barPower);
                lift.set(ControlMode.PercentOutput, liftPower);
                break;
                case STOPPED:
                move(0);
                lift.set(ControlMode.PercentOutput, 0);
                break;
        }
    }
    public void move(double power){           
        barMotorL.set(ControlMode.PercentOutput,barPower);
        barMotorR.set(ControlMode.PercentOutput,barPower);
    }
    public void setLift(double power){
        liftPower = power;
    }
    public void setPower(double power){
        indexState=States.ON;
        barPower = power;
        // indexSolStatus = !indexSolStatus;
    }

}