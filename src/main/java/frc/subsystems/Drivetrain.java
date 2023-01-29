package frc.subsystems;

import frc.robot.RobotMap;
import frc.settings.DrivetrainSettings;
import frc.util.NemesisModule;
// import com.swervedrivespecialties.swervelib.Nemesis.GearRatio;
// import frc.util.NemesisSDSWrapper.NemesisSwerveHelper;
import frc.util.NemesisSDSWrapper.NemesisSwerveHelper.GearRatio;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import java.util.function.DoubleSupplier;

/**
 * Nemesis Drivetrain for 2023
 * @author Abhik Ray
 */
public class Drivetrain implements RobotMap, Subsystem, DrivetrainSettings {

    private static Drivetrain driveTrainInstance = null;

    private Timer timer;
    private double maxAngularVelocity; // Angular Velocity of the drivetrain in radians    
    private Pigeon2 gyro;
    private ProfiledPIDController levelController;
    private ProfiledPIDController steerController;
    private ProfiledPIDController alignController;
    private final NemesisModule frontLeftModule;
    private final NemesisModule frontRightModule;
    private final NemesisModule backLeftModule;
    private final NemesisModule backRightModule;
    private final HolonomicDriveController driveController;

    private ChassisSpeeds driveSpeeds;
    private ChassisSpeeds pathfollowSpeeds;
    private ChassisSpeeds levelSpeeds;
    private ChassisSpeeds steerSpeed;
    private ChassisSpeeds alignSpeeds;
  
    public static Drivetrain getDriveInstance(PowerDistribution pdp) {
        if (driveTrainInstance == null) {
            driveTrainInstance = new Drivetrain(pdp);
        }
        return driveTrainInstance;
    }
    private enum States {
        STOPPED, DRIVE, TRAJECTORY, LEVELING, TARGETTING,
        ALIGNING
    }
    private States driveState;
    private final SwerveDriveOdometry odometry;
    // private final SwerveDrivePoseEstimator poseEstimator;
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
        // constructor 
        timer = new Timer();
        maxAngularVelocity = MAX_VELOCITY / 
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        gyro = new Pigeon2(DRIVETRAIN_PIGEON_ID,CAN_BUS);
        driveSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
        pathfollowSpeeds = new ChassisSpeeds(0.0,0.0,0.0);
        driveController = new HolonomicDriveController(
            new PIDController(0.1, 0, 0), // TBD
            new PIDController(0.1, 0, 0),  // TBD
            new ProfiledPIDController(0.2, 0,0, new Constraints(2, 0.2) //TBD
        ));
        levelController = new ProfiledPIDController(0.1, 0, 0, new Constraints(0.3, 0.1));
        steerController = new ProfiledPIDController(0.1, 0, 0, new Constraints(0.5, 0.1));
        alignController = new ProfiledPIDController(0.1, 0, 0, new Constraints(0.5, 0.1));
        // poseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, getHeadingRot(), positions, new Pose2d(0,0, new Rotation2d()));
        driveState = States.STOPPED;
        frontLeftModule = new NemesisModule(
            GearRatio.FAST, 
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET,
            "Front Left Module",
            0
        );
        frontRightModule = new NemesisModule(
            GearRatio.FAST, 
            FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
            FRONT_RIGHT_MODULE_STEER_MOTOR, 
            FRONT_RIGHT_MODULE_STEER_ENCODER, 
            FRONT_RIGHT_MODULE_STEER_OFFSET,
            "Front Right Module",
            1
        );
        backLeftModule = new NemesisModule(
            GearRatio.FAST,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER, 
            BACK_LEFT_MODULE_STEER_OFFSET, 
            "Back Left Module", 
            2
        );
        backRightModule = new NemesisModule(
            GearRatio.FAST,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER, 
            BACK_RIGHT_MODULE_STEER_OFFSET, 
            "Back Right Module", 
            3
        );
        
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

        frontLeft = new CANCoder(9,CAN_BUS);
        frontRight = new CANCoder(10,CAN_BUS);
        backLeft = new CANCoder(12,CAN_BUS);
        backRight = new CANCoder(11,CAN_BUS);
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
            case LEVELING:
                drive(levelSpeeds);
                break;
            case TARGETTING:
                // Rotation is controlled, speeds are independent 
                drive(steerSpeed);
                break;
            case TRAJECTORY:
                System.out.println("IN TRAJECTORY");
                drive(pathfollowSpeeds);
                // PathContainer.moveForward.runPath(this);
                break;
            case ALIGNING: 
                drive(alignSpeeds);
                break; 
        }
        updatePose();
    }
    public void aligning(double xOffset){
        driveState = States.ALIGNING;
        alignSpeeds = new ChassisSpeeds(
            alignController.calculate(xOffset,0),0,0);
    }
    /**
     * Sets drivetrain speeds based on a desired position from Holonomic Controller
     * @param desiredPosition
     */
    public void followPath(State desiredPosition){
        ChassisSpeeds calculatedCommand = driveController.calculate(odometry.getPoseMeters(), desiredPosition, new Rotation2d(0));
        pathfollowSpeeds = calculatedCommand;
    }
    /**
     * Switch the robot the leveling state and control position based off of Gyro Pitch 
     */
    public void autoLevel(){
        driveState = States.LEVELING;
        levelSpeeds = new ChassisSpeeds(levelController.calculate(gyro.getPitch(), 0),0,0);
    }
    /**
     * Updates each Swerve Module's Position
     */
    private void updateSwerveModules(){
        for( NemesisModule module : swerveMods) module.update();
        positions = getSwervePositions();
    }
    /**
     * Updates Robot's Odometry Position from Swerve Modules
     */
    public void updatePose(){
        updateSwerveModules();
        odometry.update(getHeadingRot(),positions);
        // poseEstimator.update(getHeadingRot(),positions);
        // poseEstimator.addVisionMeasurement(null, BACK_LEFT_MODULE_DRIVE_MOTOR, null);
    }
    /**
     * Obtains Array of Positions from Each Swerve Module
     * @return Array of Swerve Module Positions
     */
    public SwerveModulePosition[] getSwervePositions(){
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition(),

        };
    }
    /**
     * Logs Odometry Readings to Shufflebaord 
     * X, Y, Rotation (Odometry), Rotation (Gyro) 
     */
    public void outputOdometry(){
        SmartDashboard.putNumber("X Odometry", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y Odometry", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Rotation Odometry", odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Rotation Gyro", gyro.getYaw());
        // SmartDashboard.putNumber("Gyro Z", gyro.get());
    }
    /**
     * Converts Joystick input to Field Relative Drivetrain Command 
     * @param leftx Left Joystick X Input (X Motion)
     * @param lefty Left Joystick Y Input (Y Motion)
     * @param rightx Right Joystick X Input (Turn)
     */
    public void inputHandler(double leftx, double lefty, double rightx) {
        driveState = States.DRIVE;
        double x = leftx * MAX_VELOCITY;
        double y = lefty * MAX_VELOCITY;
        double angle = rightx * maxAngularVelocity;
        System.out.println(angle);
        driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, angle, getHeadingRot());//jeevan and vidur contribution to this
    }
    /**
     * Applies calculated velocities to each Swerve Module 
     * @param speeds Desired speeds of each module
     */
    public void drive(ChassisSpeeds speeds){
        states = swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);
        for(NemesisModule module : swerveMods){
            // if(module.getID() == 0) System.out.println(states[module.getID()].angle.getDegrees());
            module.set(
                (states[module.getID()].speedMetersPerSecond / MAX_VELOCITY)* MAX_VOLTAGE,  // Speed of current state, converted to voltage
                states[module.getID()].angle.getRadians() // Angle of coresponding state 
            ); 
        }
    }
    /**
     * Input Handler while Locking onto a Vision Target
     * @param leftx X Translation Field Relative
     * @param lefty Y Translation Field Relative
     * @param tx Angular Offset on X directions
     */
    public void inputHandlerTargetting(double leftx, double lefty, double tx){
        driveState = States.TARGETTING;
        double x = leftx * MAX_VELOCITY;
        double y = lefty * MAX_VELOCITY;
        double calculatedAngularCommand = steerController.calculate(tx,0) * maxAngularVelocity;
        steerSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, calculatedAngularCommand, getHeadingRot());//jeevan and vidur contribution to this

    }
    /**
     * Reset Gyro
     * @param angle the angle the gyro will reset to
     * @default Default value is 0
     */
    public void resetGyro(double angle) {
       gyro.setYaw(angle);
    }
    public void resetGyro() {
        gyro.setYaw(0);
     }
    /**
     * Set Odometry to a particular starting pose 
     * @param resetpose Starting Pose 
     * @param modPositions Starting Module Positions 
     */
    public void resetOdometry(Pose2d resetpose, SwerveModulePosition[] modPositions){
        for(NemesisModule module : swerveMods) module.reset();  
        odometry.resetPosition(getHeadingRot(),modPositions,resetpose);
    }
    /**
     * Set Encoder, Module, and Odometry Postiions to 0 
     */
    public void resetEncoder(){
        // Resetting Encoders 
        for(CANCoder encoder : encoders)encoder.setPosition(0);
        for(NemesisModule module : swerveMods) module.reset();
        resetOdometry(new Pose2d(0, 0, getHeadingRot()), positions);
        
    }
    /**
     * @return Robot Heading in Degrees
     */
    public double getHeading() {
        return gyro.getYaw(); 
    }
    /**
     * @return Robot Heading as Rotation2d Object
     */
    public Rotation2d getHeadingRot(){
        return Rotation2d.fromDegrees(gyro.getYaw());
    }
    /**
     * Set State to TRAJECTORY to follow paths 
     */
    public void startAuton(){
        driveState = States.TRAJECTORY;
    }
    /**
     * Stop Drive Train
     */
    public void stop(){
        driveState = States.STOPPED;
    }
}