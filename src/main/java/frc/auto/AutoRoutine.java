package frc.auto;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Base auto stuff
 * 
 * @author Connor_Hofenbitzer
 *
 */
public abstract class AutoRoutine {
  public abstract String getKey();

  public abstract void initialize();

  public abstract void update();

  public abstract void end();
} 