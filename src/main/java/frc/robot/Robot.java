// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auto.AutoChooser;
import frc.auto.routines.DriveSpin;
import frc.looper.Looper;
import frc.settings.FieldSettings;
import frc.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.util.NemesisJoystick;
import frc.util.Limelight;

/**
 * Code for the 2023 Nemesis Robot
 * @author Abhik Ray 
 */
public class Robot extends TimedRobot implements FieldSettings {

  public static PowerDistribution pdp;
  public static Drivetrain drivetrain;

  private NemesisJoystick leftStick;
  private NemesisJoystick rightStick;
  private AutoChooser chooser;
  private Limelight limelight;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    limelight= new Limelight();
    drivetrain = Drivetrain.getDriveInstance(pdp);

    chooser = new AutoChooser(new DriveSpin());

    addPeriodic(() -> {
      drivetrain.update();
      limelight.update();
    }, REFRESH_RATE, 0.005);

    // enabledLooper = new Looper(0.02); 
    // enabledLooper.register(drivetrain::update);
    // enabledLooper.startLoops();

    leftStick = new NemesisJoystick(0, 0.1, 0.1);
    rightStick = new NemesisJoystick(1, 0.1, 0.1);
    drivetrain.resetGyro();
    drivetrain.outputOdometry();
    drivetrain.resetEncoder();

    PathPlannerServer.startServer(5811);

  }
  @Override
  public void robotPeriodic(){
    drivetrain.outputOdometry(); 
  }

  @Override
  public void autonomousInit() {
    drivetrain.startAuton();

    // pick Auto
    chooser.pickAuto("driveSpin");
    chooser.initializeAuto();   
  }

  public void autonomousPeriodic() {
      // update Auto
      chooser.updateAuto();
  }


  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    if(rightStick.getTrigger()){

    } else {
      drivetrain.inputHandler(-leftStick.getYBanded() / 1, -leftStick.getXBanded() / 1, -rightStick.getXBanded() / 1.5);
    }

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

}
