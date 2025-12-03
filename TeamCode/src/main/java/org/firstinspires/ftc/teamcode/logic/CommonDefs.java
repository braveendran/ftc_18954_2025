package org.firstinspires.ftc.teamcode.logic;

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

    public final static double LIMELIGHT_HEADING_SHOOT_TOLERANCE_CLOSE=1.0;
    public final static double LIMELIGHT_HEADING_SHOOT_TOLERANCE_FAR=1.0;
    public final static double LIMELIGHT_HEADING_SHOOT_CLOSE_HEADING=9.7;
    public final static double LIMELIGHT_HEADING_SHOOT_FAR_HEADING=5.5;

    public final static double LIMELIGHT_HEADING_TARGETAREA_THRESHOLD= 0.5;



    public final static boolean LOCALIZER_CHECK_DISTANCE_MATCH=false;



    // Physical robot dimensions (in inches)
    // Distance between left and right wheel centers (track width)
    public static final double WHEEL_TRACK_INCHES = 18.0; // adjust to your robot's measured value
    // Distance between front and back wheel centers (wheelbase)
    public static final double WHEEL_BASE_INCHES = 18.0; // adjust to your robot's measured value

    public static final double HEADING_TOLERANCE_DEG=1.0;


    public static double ConvertCameraPosToInches_x(double x)
    {
        //convert metres to inches
        return  (x*39.37);
    }
    public static double ConvertCameraPosToInches_y(double y)
    {
        //convert metres to inches
        return  (y*39.37);
    }

    public static double ConvertCameraPosToInches_z(double z)
    {
        //convert metres to inches
        return  (z*39.37);
    }



}

