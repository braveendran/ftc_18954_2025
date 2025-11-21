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

    // Physical robot dimensions (in inches)
    // Distance between left and right wheel centers (track width)
    public static final double WHEEL_TRACK_INCHES = 18.0; // adjust to your robot's measured value
    // Distance between front and back wheel centers (wheelbase)
    public static final double WHEEL_BASE_INCHES = 18.0; // adjust to your robot's measured value

    public static final double HEADING_TOLERANCE_DEG=1.0;

}

