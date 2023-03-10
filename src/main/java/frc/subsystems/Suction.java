package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotMap;
import frc.settings.SuctionSettings;

public class Suction implements RobotMap, Subsystem, SuctionSettings {
    
    private Solenoid suctionSol;
    private Solenoid liftSol;
    private Solenoid thrustSol;
    private boolean suctionStatus;
    private boolean liftStatus;
    private boolean thrustStatus;
    private double vacuumPower = 0.5;
    private CANSparkMax vacuum;

    private static Suction succ = null;
    private enum States {
        STOPPED, ON,
    }
    private States suctionState;

    public static Suction getSuctionInstance(PowerDistribution pdp) {
        if (succ == null) {
            succ = new Suction(pdp);
        }
        return succ;
    }
    public void update(){
        switch(suctionState){
            case ON:
                outputSucc();  
                vacuum.set(vacuumPower);
                suctionSol.set(suctionStatus);
                liftSol.set(liftStatus);
                thrustSol.set(thrustStatus);
                break;
            case STOPPED:
                vacuum.set(0);
                suctionSol.set(false);
                liftSol.set(false);
                thrustSol.set(false);
        }
    }
    public void outputSucc(){
        SmartDashboard.putBoolean("Suction status", suctionStatus);
        SmartDashboard.putBoolean("Lift status", liftStatus);
        SmartDashboard.putBoolean("Thrust status", thrustStatus);
    }
    public void liftToggle(){
        suctionState = States.ON;
        liftStatus = !liftStatus;
    }
    public void succToggle(){
        suctionState = States.ON;
        suctionStatus = !suctionStatus;
        vacuumPower = suctionStatus ? SUCC_SPEED:0;
    }
    public void thrustToggle(){
        suctionState = States.ON;
        thrustStatus = !thrustStatus;
    }
    public void stop(){
        suctionState=States.STOPPED;
    }
    public Suction(PowerDistribution pdp){
        suctionState = States.STOPPED;
        suctionSol = new Solenoid(PneumaticsModuleType.CTREPCM, SUCTION_SOLENOID_ID);
        liftSol = new Solenoid(PneumaticsModuleType.CTREPCM, LIFT_SOLENOID_ID);
        thrustSol = new Solenoid(PneumaticsModuleType.CTREPCM, IN_OUT);
        vacuum = new CANSparkMax(VACUUM_ID, MotorType.kBrushless);
        liftStatus = false;
        suctionStatus = false;
        thrustStatus = false;
    }
}