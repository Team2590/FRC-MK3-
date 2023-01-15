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
import frc.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.util.NemesisJoystick;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static PowerDistribution pdp;
  public static Drivetrain drivetrain;
  private Looper enabledLooper;
  private NemesisJoystick leftStick;
  private NemesisJoystick rightStick;
  private AutoChooser chooser;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = Drivetrain.getDriveInstance(pdp);

    chooser = new AutoChooser(new DriveSpin());

    enabledLooper = new Looper(0.02); 
    enabledLooper.register(drivetrain::update);
    enabledLooper.startLoops();

    leftStick = new NemesisJoystick(0, 0.10, 0.10);
    rightStick = new NemesisJoystick(1, 0.1, 0.1);
    drivetrain.zeroGyro();
    drivetrain.outputOdometry();
    drivetrain.resetEncoder();

    // PathPlannerServer.startServer(2000);

  }

  @Override
  public void autonomousInit() {
    drivetrain.startAuton();

    // pick Auto
    chooser.pickAuto("driveSpin");
    chooser.initializeAuto();   
  }

  public void autonomousPeriodic() {
      drivetrain.updateOdometry();  

      // update Auto
      chooser.updateAuto();
  }


  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    drivetrain.inputHandler(-leftStick.getXBanded() / 3, leftStick.getYBanded() / 3, rightStick.getXBanded() / 3);
    drivetrain.outputOdometry(); 

  }

  @Override
  public void disabledInit() {
    drivetrain.stop();
  }

  @Override
  public void disabledPeriodic() {
    drivetrain.outputOdometry();

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
