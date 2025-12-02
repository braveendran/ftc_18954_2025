package org.firstinspires.ftc.teamcode.logic;

import com.pedropathing.geometry.Pose;
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

    public enum AutonRowsToCollect {
        ROS_1,
        ROS_2,
        ROS_3,
    }

    public enum AutonState {
        AUTON_MOVE_TO_SHOOT,
        AUTON_SHOOT,
        AUTON_MOVE_TO_COLLECT,
        AUTON_COLLECT_BALLS,
    }

    // Physical robot dimensions (in inches)
    // Distance between left and right wheel centers (track width)
    public static final double WHEEL_TRACK_INCHES = 9.0; // adjust to your robot's measured value
    // Distance between front and back wheel centers (wheelbase)
    public static final double WHEEL_BASE_INCHES = 18.0; // adjust to your robot's measured value

    public static final double HEADING_TOLERANCE_DEG=1.0;

    public static final double COORD_BLUE_FAR_X_CENTRE = 15;
    public static final double COORD_FAR_Y=128;
    public static final double ANGLE_BOT_FAR=45;

    public static final double COORD_CLOSE_X_FROM_CENTRE = 16;
    public static final double COORD_CLOSE_Y=6;
    public static final double ANGLE_BOT_CLOSE=90;

    public static final double ANGLE_BOT_FAR_SHOOTING=72;
    public static final double X_SHOOT_FAR=COORD_CLOSE_X_FROM_CENTRE;
    public static final double Y_SHOOT_FAR=COORD_CLOSE_Y + 10;

    public static final double ANGLE_BOT_CLOSE_SHOOTING=72;
    public static final double X_SHOOT_CLOSE=24;
    public static final double Y_SHOOT_CLOSE=96;

    public static final double BALL_X_POSITION_FROM_CENTER = 40;
    public static final double BALL_Y_POSITION_FROM_BOTTOM= 32;

    public static final double DIST_BETWEEN_ROWS=24;

    public static final double BALL_COLLECT_X_POSITION_FROM_CENTER = BALL_X_POSITION_FROM_CENTER + 16;


    public static final double ANGLE_BOT_FAR_AUTONPARK=0;
    public static final double X_AUTONPARK_FAR=COORD_CLOSE_X_FROM_CENTRE;
    public static final double Y_AUTONPARK_FAR=130;

    public static final double ANGLE_BOT_CLOSE_AUTONPARK=0;
    public static final double X_AUTONPARK_CLOSE=24;
    public static final double Y_AUTONPARK_CLOSE=10;





    //Define Pose to represent the starting position for each alliance and each position

    public static final Pose BLUE_FAR_START_POSE = new Pose(COORD_BLUE_FAR_X_CENTRE, COORD_FAR_Y, 180-ANGLE_BOT_FAR);
    public static final Pose RED_FAR_START_POSE = new Pose(144-COORD_BLUE_FAR_X_CENTRE, COORD_FAR_Y, ANGLE_BOT_FAR);

    public static final Pose RED_CLOSE_START_POSE = new Pose(72+COORD_CLOSE_X_FROM_CENTRE, COORD_CLOSE_Y, ANGLE_BOT_CLOSE);
    public static final Pose BLUE_CLOSE_START_POSE = new Pose(72-COORD_CLOSE_X_FROM_CENTRE, COORD_CLOSE_Y, ANGLE_BOT_CLOSE);




    //Define the shooting position for each alliance for each position
    public static final Pose BLUE_CLOSE_SHOOT_POSE = new Pose(72-X_SHOOT_CLOSE, Y_SHOOT_CLOSE, 180-ANGLE_BOT_CLOSE_SHOOTING);

    public static final Pose RED_CLOSE_SHOOT_POSE = new Pose(72+X_SHOOT_CLOSE, Y_SHOOT_CLOSE, ANGLE_BOT_CLOSE_SHOOTING);

    public static final Pose BLUE_FAR_SHOOT_POSE = new Pose(72+X_SHOOT_FAR, Y_SHOOT_FAR , ANGLE_BOT_FAR_SHOOTING);
    public static final Pose RED_FAR_SHOOT_POSE = new Pose(72-X_SHOOT_FAR, Y_SHOOT_FAR, 180-ANGLE_BOT_FAR_SHOOTING);


    //Define the pose position for the 3 rows of ball for red and blue
    public static final Pose BLUE_ROW1_POSE = new Pose(72-BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM, 180);
    public static final Pose BLUE_ROW2_POSE = new Pose(72-BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 180);
    public static final Pose BLUE_ROW3_POSE = new Pose(72-BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 180);
    public static final Pose RED_ROW1_POSE = new Pose(72+BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM, 0);
    public static final Pose RED_ROW2_POSE = new Pose(72+BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 0);
    public static final Pose RED_ROW3_POSE = new Pose(72+BALL_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 0);
    
    //Addz poxd forr each row position for collecting balls
    public static final Pose BLUE_ROW1_COLLECT_POSE = new Pose(72 -BALL_COLLECT_X_POSITION_FROM_CENTER , BALL_Y_POSITION_FROM_BOTTOM, 180);
    public static final Pose BLUE_ROW2_COLLECT_POSE = new Pose(72 -BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 180);
    public static final Pose BLUE_ROW3_COLLECT_POSE = new Pose(72 -BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 180);
    public static final Pose RED_ROW1_COLLECT_POSE = new Pose(72 +BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM, 0);
    public static final Pose RED_ROW2_COLLECT_POSE = new Pose(72 +BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+DIST_BETWEEN_ROWS, 0);
    public static final Pose RED_ROW3_COLLECT_POSE = new Pose(72 +BALL_COLLECT_X_POSITION_FROM_CENTER, BALL_Y_POSITION_FROM_BOTTOM+ (2*DIST_BETWEEN_ROWS), 0);


    //Parking poses at the end of auton
    public static final Pose BLUE_PARK_POSE_CLOSE = new Pose(72-X_AUTONPARK_CLOSE, Y_AUTONPARK_CLOSE, ANGLE_BOT_CLOSE_AUTONPARK);

    public static final Pose RED_PARK_POSE_CLOSE = new Pose(X_AUTONPARK_CLOSE+72, Y_AUTONPARK_CLOSE, 180-ANGLE_BOT_CLOSE_AUTONPARK);
    public static final Pose RED_PARK_POSE_FAR = new Pose(72+X_AUTONPARK_FAR, Y_AUTONPARK_FAR, ANGLE_BOT_FAR_AUTONPARK);
    public static final Pose BLUE_PARK_POSE_FAR = new Pose(72-X_AUTONPARK_FAR, Y_AUTONPARK_FAR, 180-ANGLE_BOT_FAR_AUTONPARK);



}

