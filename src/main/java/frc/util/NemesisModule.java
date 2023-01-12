package frc.util;
import java.lang.reflect.Constructor;

import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper.GearRatio;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class NemesisModule implements SwerveModule{
    SwerveModule module;
    double distance;
    public NemesisModule(GearRatio gearRatio, int driveId, int steerId, int steerEncoder, double steerOffset){
        // Add shuffleboard tab integration later 
        module = Mk3SwerveModuleHelper.createFalcon500(            
            // This can either be STANDARD or FAST depending on your gear configuration
            gearRatio,
            driveId,
            steerId,
            steerEncoder,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            steerOffset
        );
        distance = 0;
    }
    // Inherited from Swerve Module 
    public double getDriveVelocity(){
        System.out.println("Swerve Module Must be Configured Before Using getDriveVelocity");
        return 0;
    };
    public double getSteerAngle(){
        System.out.println("Swerve Module Must be Configured Before Using getSteerAngle");
        return 0;
    };
    public void set(double driveVoltage, double steerAngle){};
    // 
    public void configureShuffleBoard(){ 
        
    }
    public void update(){
        // distance = integral of velocity
        distance += getDriveVelocity() * 0.02;
    }
    public void reset(){
        distance = 0;
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(distance, new Rotation2d(getSteerAngle()));

    }

    
}
