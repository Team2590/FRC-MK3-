package frc.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.settings.FieldSettings;

/**
 * Limelight camera class for vision processing. Uses network tables relayed from
 * the camera to track retroreflective/AprilTag targets. x, y, skew, and distance
 * are smoothed.
 * 
 * @author Harsh Padhye, Chinmay Savanur, Rohan Bhatnagar, Ishan Arora, Abhik likes cat-gurlz, Elan Ronen
 * 
 *         Access the Limelight web pipeline via: http://10.25.90.11:5801
 *         Access the Limelight camera feed via: http://10.25.90.11:5800
 *         Access the Limelight IP camera in GRIP via: http://10.25.90.11:5802
 */
public class Limelight implements LimelightSettings, FieldSettings {

  //~~~~~~ SINGLETON ~~~~~~

  private static Limelight instance = null;

  public static Limelight getInstance() {
    return instance == null ? instance = new Limelight() : instance;
  }

  //~~~~~~ HAS TARGET ~~~~~~

  private boolean hasTarget; // Whether the limelight has any valid targets
  private final DoubleSubscriber tvSub;

  //~~~~~~ X ~~~~~~

  private double x; // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  private final DoubleSubscriber txSub;
  private final Smoother xSmoother = new Smoother(SAMPLE_SIZE);

  //~~~~~~ Y ~~~~~~

  private double y; // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  private final DoubleSubscriber tySub;
  private final Smoother ySmoother = new Smoother(SAMPLE_SIZE);

  //~~~~~~ SKEW ~~~~~~

  private double skew; // Skew or rotation (-90 degrees to 0 degrees)
  private final DoubleSubscriber tsSub;
  private final Smoother sSmoother = new Smoother(SAMPLE_SIZE);

  //~~~~~~ DISTANCE ~~~~~~

  private double xDist;
  private final Smoother dSmoother = new Smoother(SAMPLE_SIZE);
  private double lastDistance;
  private double deltaDistance;

  //~~~~~~ APRILTAGS ~~~~~~
  private int aprilTagId;
  private final DoubleSubscriber tidSub;

  //~~~~~~ LIMELIGHT ~~~~~~

  private boolean limelightOn;
  private final DoublePublisher ledMode;
  private double limelight_pos_x;
  private double limelight_pos_y;

  private DoublePublisher pipelinePublisher;

  //~~~~~~ CAMTRAIN ~~~~~~

  private final double[] DEFAULT_ARRAY = { 0, 0, 0, 0, 0, 0 };
  private double[] camtran = DEFAULT_ARRAY;
  private final DoubleArraySubscriber camtranSub;

  //~~~~~~ CODE ~~~~~~

  // TODO: smooth out of target or set to 0 and reset?
  // TODO: add javadocs
  // TODO: look at commands/subsystems (https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)

  public Limelight() {
    var table = NetworkTableInstance.getDefault().getTable("limelight");

    pipelinePublisher = table.getDoubleTopic("pipeline").publish();
    pipelinePublisher.set(0.0);

    txSub = table.getDoubleTopic("tx").subscribe(0);
    tySub = table.getDoubleTopic("ty").subscribe(0);
    tvSub = table.getDoubleTopic("tv").subscribe(0.0);
    tsSub = table.getDoubleTopic("ts").subscribe(0);
    camtranSub = table.getDoubleArrayTopic("camtran").subscribe(new double[0]); 
    tidSub = table.getDoubleTopic("tid").subscribe(0);
    ledMode = table.getDoubleTopic("ledMode").publish();
  }

  public void update() {
    checkForTarget();
    if (hasTarget) {
      x = xSmoother.push(txSub.get(0));
      y = ySmoother.push(tySub.get(0));
      skew = sSmoother.push(tsSub.get(0));
      xDist = dSmoother.push((HUB_TARGET_HEIGHT - CAMERA_HEIGHT) / (Math.tan(Math.toRadians(y + MOUNT_ANGLE))));
      deltaDistance = xDist - lastDistance;
      lastDistance = xDist;
      aprilTagId = (int) tidSub.get(-1);
    } else {
      x = y = skew = 0;
      aprilTagId = -1;
      xSmoother.reset();
      ySmoother.reset();
      sSmoother.reset();
      dSmoother.reset();
    }
    // camtran = camtranSub.getDoubleArray(DEFAULT_ARRAY);
  }
  /**
   * Checks to see if there are vision targets within frame
   */
  public void checkForTarget(){
    hasTarget = tvSub.get(0) > 0;
  }
  public boolean hasTarget() {
    return hasTarget;
  }

  public void outputShuffleboard(){
    SmartDashboard.putBoolean("Limelight Has Target", hasTarget);
    SmartDashboard.putNumber("April Tag Seen", aprilTagId);
    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
  }

  /**
   * Horizontal angle to the target in degrees
   * 
   * @return the horizontal angle from the camera to the target in degrees
   */
  public double getX() {
    return x;
  }

  /**
   * Vertical angle to the target in degrees
   * 
   * @return the vertical angle from the camera to the target in degrees
   */
  public double getY() {
    return y;
  }

  /**
   * Probably how rotated the target is in degrees
   * 
   * @return update this please
   */
  public double getSkew() {
    return skew;
  }

  /**
   * Distance in inches
   * 
   * @return distance in inches
   */
  public double getDistance() {
    return xDist;
  }

  public double getTrueDistance(double limelightDist) {
    double funnyNumber = 10;
    return (1.24 * limelightDist) - 12.9 + 24 + funnyNumber;
  }

  public double getDeltaDistance() {
    return deltaDistance;
  }

  public int getAprilTagID() {
    return aprilTagId;
  }

  public Pose2d getPoseLimelight(double turretHeading, double heading) {
    limelight_pos_y = getTrueDistance(getDistance()) * Math.sin(Math.toRadians(turretHeading + heading + 180));
    limelight_pos_x = getTrueDistance(getDistance()) * Math.cos(Math.toRadians(turretHeading + heading + 180));
    return new Pose2d(
      new Translation2d(limelight_pos_x / MOUNT_ANGLE, limelight_pos_y / MOUNT_ANGLE),
      new Rotation2d(Math.toRadians(heading))
    );
  }

  public double getCamTranX() {
    return hasTarget ? camtran[CAMTRAN_X] : 0;
  }

  public double getCamTranY() {
    return hasTarget ? camtran[CAMTRAN_Y] : 0;
  }

  public double getCamTranZ() {
    return hasTarget ? camtran[CAMTRAN_Z] : 0;
  }

  public double getCamTranPitch() {
    return hasTarget ? camtran[CAMTRAN_PITCH] : 0;
  }

  public double getCamTranYaw() {
    return hasTarget ? camtran[CAMTRAN_YAW] : 0;
  }

  public double getCamTranRoll() {
    return hasTarget ? camtran[CAMTRAN_ROLL] : 0;
  }

  // ?????????
  public boolean has3DLocation() {
    return (Math.abs(camtran[CAMTRAN_YAW]) > 0.0001);
  }

  /**
   * Calculates angle to become perpendicular to target
   * 
   * @return the horizontal angle between the target perpendicular and the
   *         robot-target line
   */
  public double getAngle2() {
    return getCamTranYaw() - x;
  }

  public boolean isOn() {
    return limelightOn;
  }

  public void turnOn() {
    ledMode.set(LED_ON);
    limelightOn = true;
  }

  public void turnOff() {
    ledMode.set(LED_OFF);
    limelightOn = false;
  }
}
