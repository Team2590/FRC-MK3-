
package frc.settings;

import java.util.HashMap;

public interface FieldSettings {
    public static final double REFRESH_RATE = 0.01;

    // targets (inches)
    public static final HashMap<Integer, Double> APRILTAG_HEIGHTS = new HashMap<>(2) {{ // AprilTag heights
        put(0, 1.0);
        put(1, 2.0);
    }};
    public static final double HUB_TARGET_HEIGHT = 0;

    public static final double ROBOT_NORTH = 0.70;
    public static final double ROBOT_SOUTH = 0.22;
    public static final double ROBOT_EAST = 0.46;
    public static final double ROBOT_WEST = 0.94;

    // (x,y)
    public static final double FIVE_BALL_POSE_X = 2.22;
    public static final double FIVE_BALL_POSE_Y = -0.8;

    public static final double TWO_BALL_POSE_X = 2.38;
    public static final double TWO_BALL_POSE_Y = 0.736;
}