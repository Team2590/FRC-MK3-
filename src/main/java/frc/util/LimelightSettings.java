package frc.util;

/**
 * Constants and Settings of the Limelight Camera
 */
public interface LimelightSettings {

    // dependent on the mounting of the camera, field measurements, etc.
    /**
     * in inches
     */
    public static final double CAMERA_HEIGHT = 4;
    /**
     * in degrees
     */
    public static final double MOUNT_ANGLE = 0;

    // camtran indices
    public static final int CAMTRAN_X = 0;
    public static final int CAMTRAN_Y = 1;
    public static final int CAMTRAN_Z = 2;
    public static final int CAMTRAN_PITCH = 3;
    public static final int CAMTRAN_YAW = 4;
    public static final int CAMTRAN_ROLL = 5;

    // led modes
    public static final int LED_OFF = 1;
    public static final int LED_ON = 3;

    // smooothing
    public static final int SAMPLE_SIZE = 8;
}