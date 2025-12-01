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

    public enum AutonRowsToCollect {
        ROWS_1,
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
    public static final double WHEEL_TRACK_INCHES = 18.0; // adjust to your robot's measured value
    // Distance between front and back wheel centers (wheelbase)
    public static final double WHEEL_BASE_INCHES = 18.0; // adjust to your robot's measured value

    public static final double HEADING_TOLERANCE_DEG=1.0;

    //Define Pose to represent the starting position for each alliance and each position
    public static final Pose BLUE_CLOSE_START_POSE = new Pose(12, 12, 0);
    public static final Pose BLUE_FAR_START_POSE = new Pose(12, 48, 0);
    public static final Pose RED_CLOSE_START_POSE = new Pose(12, -12, 180);
    public static final Pose RED_FAR_START_POSE = new Pose(12, -48, 180);

    //Define Pose to represent the parking positions for each alliance for each position
    public static final Pose BLUE_CLOSE_PARK_POSE = new Pose(60, 12, 0);
    public static final Pose BLUE_FAR_PARK_POSE = new Pose(60, 48, 0);
    public static final Pose RED_CLOSE_PARK_POSE = new Pose(60, -12, 180);
    public static final Pose RED_FAR_PARK_POSE = new Pose(60, -48, 180);

    //Define the shooting position for each alliance for each position
    public static final Pose BLUE_CLOSE_SHOOT_POSE = new Pose(36, 12, 0);
    public static final Pose BLUE_FAR_SHOOT_POSE = new Pose(36, 48, 0);
    public static final Pose RED_CLOSE_SHOOT_POSE = new Pose(36, -12, 180);
    public static final Pose RED_FAR_SHOOT_POSE = new Pose(36, -48, 180);


    //Define the pose position for the 3 rows of ball for red and blue
    public static final Pose BLUE_ROW1_POSE = new Pose(30, 24, 0);
    public static final Pose BLUE_ROW2_POSE = new Pose(48, 24, 0);
    public static final Pose BLUE_ROW3_POSE = new Pose(66, 24, 0);
    public static final Pose RED_ROW1_POSE = new Pose(30, -24, 180);
    public static final Pose RED_ROW2_POSE = new Pose(48, -24, 180);
    public static final Pose RED_ROW3_POSE = new Pose(66, -24, 180);   
    
    //Addz poxd forr each row position for collecting balls
    public static final Pose BLUE_ROW1_COLLECT_POSE = new Pose(30, 30, 0);
    public static final Pose BLUE_ROW2_COLLECT_POSE = new Pose(48, 30, 0);
    public static final Pose BLUE_ROW3_COLLECT_POSE = new Pose(66, 30, 0);
    public static final Pose RED_ROW1_COLLECT_POSE = new Pose(30, -30, 180);
    public static final Pose RED_ROW2_COLLECT_POSE = new Pose(48, -30, 180);
    public static final Pose RED_ROW3_COLLECT_POSE = new Pose(66, -30, 180);



}

