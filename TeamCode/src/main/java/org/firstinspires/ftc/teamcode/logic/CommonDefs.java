package org.firstinspires.ftc.teamcode.logic;

// import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower; // REMOVED: You should not reference follower here
// import com.pedropathing.geometry.BezierLine; // REMOVED: Not needed in this file
import com.pedropathing.geometry.Pose;
// import java.nio.file.Path; // REMOVED: Incorrect import for robot paths

/**
 * Common definitions and enums used across the autonomous system
 */
public class CommonDefs {

    public enum Alliance {
        BLUE,
        RED
    }

    public enum PositionType {
        CLOSE,
        FAR
    }

    public final static int RED_TAG_ID = 24;
    public final static int BLUE_TAG_ID = 20;

    // --- LIMELIGHT CONSTANTS ---
    public final static double LIMELIGHT_HEADING_SHOOT_TOLERANCE_CLOSE=1.0;
    public final static double LIMELIGHT_HEADING_SHOOT_TOLERANCE_FAR=0.5;
    public final static double LIMELIGHT_HEADING_SHOOT_CLOSE_HEADING=(9.7-3.8);
    public final static double LIMELIGHT_HEADING_SHOOT_FAR_HEADING_BLUE=2.0;
    public final static double LIMELIGHT_HEADING_SHOOT_FAR_HEADING_RED=-1.0;
    public final static double LIMELIGHT_HEADING_TARGETAREA_THRESHOLD= 0.5;
    public final static boolean LOCALIZER_CHECK_DISTANCE_MATCH=false;

    // --- ROBOT PHYSICAL CONSTANTS ---
    // Distance between left and right wheel centers (track width)
    public static final double WHEEL_TRACK_INCHES = 18.0; // adjust to your robot's measured value
    // Distance between front and back wheel centers (wheelbase)
    public static final double WHEEL_BASE_INCHES = 18.0; // adjust to your robot's measured value
    public static final double HEADING_TOLERANCE_DEG=1.0;


    // --- CONVERSION UTILITIES ---
    public static double ConvertCameraPosToInches_x(double x) {
        //convert metres to inches
        return (x * 39.37);
    }
    public static double ConvertCameraPosToInches_y(double y) {
        //convert metres to inches
        return (y * 39.37);
    }
    public static double ConvertCameraPosToInches_z(double z) {
        //convert metres to inches
        return (z * 39.37);
    }

    // --- EXAMPLE POSE DEFINITIONS ---
    public static final Pose START_POSE_EXAMPLE = new Pose(119, 129, Math.toRadians(45));
    public static final Pose END_POSE_EXAMPLE = new Pose(84, 84, Math.toRadians(45));

    //
    // THE PATH BUILDING LOGIC HAS BEEN REMOVED FROM THIS FILE.
    // IT MUST BE PLACED INSIDE YOUR AUTONOMOUS OPMODE'S runOpMode() METHOD.
    //
}
