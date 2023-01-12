package frc.subsystems;

import frc.robot.RobotMap;
import frc.util.NemesisModule;
import frc.auto.trajectory.PathContainer;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
// import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.sensors.CANCoder;

import java.nio.file.Path;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Drivetrain implements RobotMap, Subsystem, DrivetrainSettings {

    private static Drivetrain driveTrainInstance = null;
    private double maxVoltage;
    private double maxVelocity; // Meters per second
    private double maxAngularVelocity; // Angular Velocity of the drivetrain in radians    
    private Pigeon2 gyro;
    private final NemesisModule frontLeftModule;
    private final NemesisModule frontRightModule;
    private final NemesisModule backLeftModule;
    private final NemesisModule backRightModule;
    private final HolonomicDriveController driveController;
    private DoubleSupplier translationXSupp;
    private DoubleSupplier translationYSupp;
    private DoubleSupplier rotationSupp;

    private ChassisSpeeds driveSpeeds;
    private ChassisSpeeds pathfollowSpeeds;

    private ShuffleboardTab mainTab;
  
    public static Drivetrain getDriveInstance(PowerDistribution pdp) {
        if (driveTrainInstance == null) {
            driveTrainInstance = new Drivetrain(pdp);
        }
        return driveTrainInstance;
    }
    private enum States {
        STOPPED, DRIVE, TRAJECTORY
    }
    private States driveState;
    private final SwerveDriveOdometry odometry;
    private SwerveModuleState[] states;
    private SwerveModulePosition[] positions;
    CANCoder frontLeft;
    CANCoder frontRight;
    CANCoder backLeft;
    CANCoder backRight;
    public NemesisModule[] swerveMods;
    public CANCoder[] encoders;

    public DoubleSupplier odometryX; 
    public DoubleSupplier odometryY; 
    public DoubleSupplier odometryRot;

    
    public Drivetrain(PowerDistribution pdp) {
        
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        mainTab = Shuffleboard.getTab("Main");
        // constructor 
        maxVoltage = 12;
        maxVelocity = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

        maxAngularVelocity = maxVelocity / 
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        gyro = new Pigeon2(DRIVETRAIN_PIGEON_ID);
        driveSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
        pathfollowSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
        driveController = new HolonomicDriveController(
            new PIDController(0.1, 0, 0), 
            new PIDController(0.1, 0, 0), 
            new ProfiledPIDController(0.2, 0,0, new Constraints(2, 0.2)
        ));
        driveState = States.STOPPED;
        frontLeftModule = new NemesisModule(
            GearRatio.STANDARD, 
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_DRIVE_MOTOR
        );
        frontRightModule = new NemesisModule(
            GearRatio.STANDARD, 
            FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
            FRONT_RIGHT_MODULE_STEER_MOTOR, 
            FRONT_RIGHT_MODULE_STEER_ENCODER, 
            FRONT_RIGHT_MODULE_DRIVE_MOTOR
        );
        backLeftModule = new NemesisModule(GearRatio.STANDARD, BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_DRIVE_MOTOR);
        backRightModule = new NemesisModule(GearRatio.STANDARD, BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_DRIVE_MOTOR);
        
        positions = new SwerveModulePosition[] 
            {frontLeftModule.getPosition(), frontRightModule.getPosition(), 
            backLeftModule.getPosition(), backRightModule.getPosition()};

        odometry = new SwerveDriveOdometry(swerveKinematics, getHeadingRot(), positions, new Pose2d(0,0, new Rotation2d()));
        odometryX = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return odometry.getPoseMeters().getX();
            }
        };
        odometryY = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return odometry.getPoseMeters().getY();
            }
        };
        odometryRot = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return odometry.getPoseMeters().getRotation().getDegrees();
            }
        };

        frontLeft = new CANCoder(9);
        frontRight = new CANCoder(10);
        backLeft = new CANCoder(12);
        backRight = new CANCoder(11);
        swerveMods = new NemesisModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};
        encoders = new CANCoder[]{frontLeft, frontRight, backLeft, backRight};
        
    }
    public void update(){
        switch(driveState){
            case DRIVE:
                drive(driveSpeeds);
                break;
            case STOPPED:                
                drive(new ChassisSpeeds(0,0,0));
                break;
            case TRAJECTORY:
                System.out.println("IN TRAJECTORY");
                drive(pathfollowSpeeds);
                // PathContainer.moveForward.runPath(this);
                break;
        }
        updateOdometry();
    }
    public void followPath(State desiredPosition){
        ChassisSpeeds calculatedCommand = driveController.calculate(odometry.getPoseMeters(), desiredPosition, new Rotation2d(0));
        pathfollowSpeeds = calculatedCommand;
    }
    public void updateOdometry(){
        for( NemesisModule module : swerveMods){
            module.update();
        }
        positions = getSwervePositions();
        odometry.update(getHeadingRot(),positions);
    }
    public SwerveModulePosition[] getSwervePositions(){
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition(),

        };
    }

    public void outputOdometry(){
        SmartDashboard.putNumber("X Odometry", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y Odometry", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("rotation Odometry", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Rotation Gyro", gyro.getYaw());
        // SmartDashboard.pu
    }

    public void inputHandler(double leftx, double lefty, double rightx) {
        driveState = States.DRIVE;
        double x = leftx * maxVelocity;
        double y = lefty * maxVelocity;
        double angle = rightx * maxAngularVelocity;
        driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, angle, getHeadingRot());//jeevan and vidur contribution to this
       //driveSpeeds = new ChassisSpeeds(x, y, angle);
        System.out.println(driveSpeeds);
        System.out.printf("X: %f Y: %f Angle: %f\n",x,y,angle); 
    }
    public void drive(ChassisSpeeds speeds){
        states = swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);
        for(NemesisModule module : swerveMods) module.reset();  
        frontLeftModule.set((states[0].speedMetersPerSecond / maxVelocity)* maxVoltage, states[0].angle.getRadians());
        frontRightModule.set((states[1].speedMetersPerSecond / maxVelocity) * maxVoltage, states[1].angle.getRadians());
        backLeftModule.set((states[2].speedMetersPerSecond / maxVelocity) * maxVoltage, states[2].angle.getRadians());
        backRightModule.set((states[3].speedMetersPerSecond / maxVelocity) * maxVoltage, states[3].angle.getRadians());
        
    }
    public void zeroGyro() {
       gyro.setYaw(0);
      }
    public void resetOdometry(Pose2d resetpose, SwerveModulePosition[] modPositions){
        odometry.resetPosition(getHeadingRot(),modPositions,resetpose);

    }
    public void resetEncoder(){
        // Resetting Encoders 
        for(CANCoder encoder : encoders){
            encoder.setPosition(0);
        }
        for(NemesisModule module : swerveMods){
            module.reset();
        }
        resetOdometry(new Pose2d(0, 0, getHeadingRot()), positions);
        
    }
    
    public double getHeading() {
        // return new Rotation2d(gyro.getYaw()).getDegrees();
        return gyro.getYaw(); 
    }
    public Rotation2d getHeadingRot(){
        return Rotation2d.fromDegrees(gyro.getYaw());
    }
    public void startAuton(){
        driveState = States.TRAJECTORY;
    }
    public void stop(){
        driveState = States.STOPPED;
    }
}