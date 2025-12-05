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
    public final static double LIMELIGHT_HEADING_SHOOT_CLOSE_HEADING=(9.7-3.8);
    public final static double LIMELIGHT_HEADING_SHOOT_FAR_HEADING=(5.5-6.5);

    public final static double LIMELIGHT_HEADING_TARGETAREA_THRESHOLD= 0.5;

    // Distance threshold for "too close" warning (inches)
    public final static double TARGET_TOO_CLOSE_THRESHOLD_INCHES = 24.0;

    public final static boolean LOCALIZER_CHECK_DISTANCE_MATCH=false;
    
    // End game parking bounding boxes (x_min, y_min, x_max, y_max in inches)
    // Red alliance end game parking area
    public static final double RED_ENDGAME_PARK_X_MIN = 108.0;
    public static final double RED_ENDGAME_PARK_Y_MIN = 108.0;  
    public static final double RED_ENDGAME_PARK_X_MAX = 144.0;
    public static final double RED_ENDGAME_PARK_Y_MAX = 144.0;
    
    // Blue alliance end game parking area
    public static final double BLUE_ENDGAME_PARK_X_MIN = 0.0;
    public static final double BLUE_ENDGAME_PARK_Y_MIN = 108.0;
    public static final double BLUE_ENDGAME_PARK_X_MAX = 36.0;
    public static final double BLUE_ENDGAME_PARK_Y_MAX = 144.0;



    // Physical robot dimensions (in inches)
    // Distance between left and right wheel centers (track width)
    public static final double WHEEL_TRACK_INCHES = 16.0; // adjust to your robot's measured value
    // Distance between front and back wheel centers (wheelbase)
    public static final double WHEEL_BASE_INCHES = 8.0; // adjust to your robot's measured value

    public static final double HEADING_TOLERANCE_DEG=1.0;


     public static final double COORD_FAR_X_CENTRE = 15;
    public static final double COORD_FAR_Y=6;
    public static final double ANGLE_BOT_FAR=90;


    public static final double COORD_CLOSE_X_FROM_CENTRE = 57;
    public static final double COORD_CLOSE_Y=128;
    public static final double ANGLE_BOT_CLOSE=45;

    public static final double ANGLE_BOT_FAR_SHOOTING=72;
    public static final double X_SHOOT_FAR=COORD_FAR_X_CENTRE;
    public static final double Y_SHOOT_FAR=COORD_FAR_Y + 10;

    public static final double ANGLE_BOT_CLOSE_SHOOTING=72;
    public static final double X_SHOOT_CLOSE=24;
    public static final double Y_SHOOT_CLOSE=96;

    public static final double BALL_X_POSITION_FROM_CENTER = 40;
    public static final double BALL_Y_POSITION_FROM_BOTTOM= 32;

    public static final double DIST_BETWEEN_ROWS=24;

    public static final double BALL_COLLECT_X_POSITION_FROM_CENTER = BALL_X_POSITION_FROM_CENTER + 16;


    public static final double ANGLE_BOT_FAR_AUTONPARK=0;
    public static final double X_AUTONPARK_FAR=30;
    public static final double Y_AUTONPARK_FAR=10;

    public static final double ANGLE_BOT_CLOSE_AUTONPARK=0;
    public static final double X_AUTONPARK_CLOSE=10;
    public static final double Y_AUTONPARK_CLOSE=125;





    //Define Pose to represent the starting position for each alliance and each position

    public static final Pose BLUE_FAR_START_POSE = new Pose(72-COORD_FAR_X_CENTRE, COORD_FAR_Y, ANGLE_BOT_FAR);
    public static final Pose RED_FAR_START_POSE = new Pose(72+COORD_FAR_X_CENTRE, COORD_FAR_Y, ANGLE_BOT_FAR);

    public static final Pose RED_CLOSE_START_POSE = new Pose(72+COORD_CLOSE_X_FROM_CENTRE, COORD_CLOSE_Y, ANGLE_BOT_CLOSE);
    public static final Pose BLUE_CLOSE_START_POSE = new Pose(72-COORD_CLOSE_X_FROM_CENTRE, COORD_CLOSE_Y, 180-ANGLE_BOT_CLOSE);




    //Define the shooting position for each alliance for each position
    public static final Pose BLUE_CLOSE_SHOOT_POSE = new Pose(72-X_SHOOT_CLOSE, Y_SHOOT_CLOSE, 180-ANGLE_BOT_CLOSE_SHOOTING);

    public static final Pose RED_CLOSE_SHOOT_POSE = new Pose(72+X_SHOOT_CLOSE, Y_SHOOT_CLOSE, ANGLE_BOT_CLOSE_SHOOTING);

    public static final Pose BLUE_FAR_SHOOT_POSE = new Pose(72-X_SHOOT_FAR, Y_SHOOT_FAR , ANGLE_BOT_FAR_SHOOTING);
    public static final Pose RED_FAR_SHOOT_POSE = new Pose(72+X_SHOOT_FAR, Y_SHOOT_FAR, 180-ANGLE_BOT_FAR_SHOOTING);


    //Define the pose position for the 3 rows of ball for red and blue
    public static final Pose BLUE_ROW3_POSE = new Pose(72-BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM, 180);
    public static final Pose BLUE_ROW2_POSE = new Pose(72-BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 180);
    public static final Pose BLUE_ROW1_POSE = new Pose(72-BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 180);
    public static final Pose RED_ROW3_POSE = new Pose(72+BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM, 0);
    public static final Pose RED_ROW2_POSE = new Pose(72+BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 0);
    public static final Pose RED_ROW1_POSE = new Pose(72+BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 0);
    
    //Addz poxd forr each row position for collecting balls
    public static final Pose BLUE_ROW3_COLLECT_POSE = new Pose(72 -BALL_COLLECT_X_POSITION_FROM_CENTER , BALL_Y_POSITION_FROM_BOTTOM, 180);
    public static final Pose BLUE_ROW2_COLLECT_POSE = new Pose(72 -BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 180);
    public static final Pose BLUE_ROW1_COLLECT_POSE = new Pose(72 -BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 180);
    public static final Pose RED_ROW3_COLLECT_POSE = new Pose(72 +BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM, 0);
    public static final Pose RED_ROW2_COLLECT_POSE = new Pose(72 +BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 0);
    public static final Pose RED_ROW1_COLLECT_POSE = new Pose(72 +BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 0);


    //Parking poses at the end of auton
    public static final Pose BLUE_PARK_POSE_CLOSE = new Pose(72-X_AUTONPARK_CLOSE, Y_AUTONPARK_CLOSE, ANGLE_BOT_CLOSE_AUTONPARK);

    public static final Pose RED_PARK_POSE_CLOSE = new Pose(X_AUTONPARK_CLOSE+72, Y_AUTONPARK_CLOSE, 180-ANGLE_BOT_CLOSE_AUTONPARK);
    public static final Pose RED_PARK_POSE_FAR = new Pose(72+X_AUTONPARK_FAR, Y_AUTONPARK_FAR, ANGLE_BOT_FAR_AUTONPARK);
    public static final Pose BLUE_PARK_POSE_FAR = new Pose(72-X_AUTONPARK_FAR, Y_AUTONPARK_FAR, 180-ANGLE_BOT_FAR_AUTONPARK);



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

