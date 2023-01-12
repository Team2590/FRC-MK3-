package frc.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoChooser {

  private AutoRoutine[] autoHolder;
  private AutoRoutine currentSelected;

  /**
   * Handles possible auto routines we could run
   * 
   * @param autos : all of our autos
   */
  public AutoChooser(AutoRoutine... autos) {
    autoHolder = autos;
    currentSelected = null;
  }

  /**
   * Picks the auto you have chosen
   * 
   * @param autoKey : the name of the autonomous
   */
  public boolean pickAuto(String autoKey) {
    currentSelected = null;

    // this is probably really inefficient, try to find a dynamic way to do this
    // next year
    for (AutoRoutine a : autoHolder) {

      // make it case insensitive
      if (a.getKey().toLowerCase().equals(autoKey.toLowerCase())) {
        currentSelected = a;
        return true;
      }
    }

    currentSelected = null;
    return false;
  }

  /**
   * Initializes the chosen auto
   */
  public void initializeAuto() {

    // if there is an auto to run then run that auto
    if (currentSelected != null) {
      DriverStation.reportWarning("Starting auto " + currentSelected.getKey(), false);
      currentSelected.initialize();
    } else {

      // other wise make people know that they didnt actually choose an auto
      DriverStation.reportWarning("No auto chosen, running sit still", false);
    }
  }

  /**
   * Periodically updates the chosen auto routine
   */
  public void updateAuto(){
    if (currentSelected != null) {
      currentSelected.update();
    }
  }

  /**
   * Kills the autonomous mode
   */
  public void endAuto() {
    if (currentSelected != null) {
      currentSelected.end();
      currentSelected = null;
    }
  }

}