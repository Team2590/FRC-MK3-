// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auto.AutoChooser;
import frc.auto.routines.CommunityPlacement;
import frc.auto.routines.DriveSpin;
import frc.auto.routines.CommunityPlacement;
import frc.looper.Looper;
import frc.settings.DrivetrainSettings;
import frc.settings.FieldSettings;
import frc.subsystems.BarIndexer;
import frc.subsystems.Drivetrain;
// import frc.subsystems.Suction;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.util.NemesisJoystick;
// import frc.util.GridReader;
import frc.util.Limelight;

/**
 * Code for the 2023 Nemesis Robot
 * @author Abhik Ray 
 */
public class Robot extends TimedRobot implements FieldSettings,DrivetrainSettings {

  public static PowerDistribution pdp;
  public static Drivetrain drivetrain;
  public static BarIndexer indexer;
  // public static Suction suction;

  public static Compressor compressor;
  
  private NemesisJoystick leftStick;
  private NemesisJoystick rightStick;
  private AutoChooser chooser;
  // private Limelight limelight_top, limelight_bottom;
  // private GridReader reader;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // limelight_top= new Limelight("limelight_top");
    // limelight_bottom= new Limelight("limelight_bottom");
    drivetrain = Drivetrain.getDriveInstance(pdp);
    // indexer = BarIndexer.getIndexerInstance(pdp);
    //  suction = Suction.getSuctionInstance(pdp);
    chooser = new AutoChooser(new CommunityPlacement());
    // suction = Suction.getSuctionInstance(pdp);
  
    // reader = new GridReader();

    addPeriodic(() -> {
      drivetrain.update();
      // indexer.update(); 
      // suction.update();
      // limelight_bottom.update();
    }, REFRESH_RATE, 0.005);

   
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    leftStick = new NemesisJoystick(0, 0.1, 0.1);
    rightStick = new NemesisJoystick(1, 0.1, 0.1);
    drivetrain.resetGyro();
    drivetrain.outputOdometry();
    // drivetrain.resetEncoder();

    PathPlannerServer.startServer(2590);

  }
  @Override
  public void robotPeriodic(){
    drivetrain.outputOdometry(); 
    //limelight.outputShuffleboard();
  }

  @Override
  public void autonomousInit() {
    drivetrain.startAuton();
    SmartDashboard.putNumber("max velocity", MAX_VELOCITY);
    SmartDashboard.putNumber("max acceleration", MAX_ACCELERATION);

    // pick Auto
    chooser.pickAuto("Community_Placement");
    chooser.initializeAuto();   
  }

  public void autonomousPeriodic() {
      // update Auto
      chooser.updateAuto();
  }


  @Override
  public void teleopInit() {
    compressor.enableDigital();
  }

  @Override
  public void teleopPeriodic() {
    if(leftStick.getTrigger()){
      // drivetrain.aligning(limelight_top.getX());
    } else if(rightStick.getTrigger()){
      drivetrain.autoLevel();
    }
    else {
      drivetrain.inputHandler(leftStick.getYBanded() /2, leftStick.getXBanded() /2, rightStick.getXBanded() / 1.5);
    }
    if(leftStick.getRawButtonPressed(4)){
      // reader.findConeTarget();
    }
    // if(leftStick.getTriggerPressed()){
    //   suction.liftToggle();
    // }
    // if(rightStick.getTriggerPressed()){
    //   suction.succToggle(); 
    // }
    // // if(leftStick.getRawButtonPressed(3)){
    // //   indexer.toggleIndexer();
    // // }                                                     
    // if(rightStick.getRawButtonPressed(3)){
    //   suction.thrustToggle();
    // }
  }

  @Override
  public void disabledInit() {
    drivetrain.stop();  
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static Drivetrain getDrivetrainInstance(){
    return drivetrain;
  }
  // public static Suction getSuctionInstance(){
  //   return suction;
  // }
  public static BarIndexer getIndexerInstance(){
    return indexer;
  }
}
