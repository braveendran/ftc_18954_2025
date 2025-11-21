package org.firstinspires.ftc.teamcode.params;

public class AutonFarParams {


    public final long LAUNCHER_POS1_RPM = 3600;

    public final long LAUNCHER_BACKSHOOT_COMENEAR_RPM=2900;


    public final double BALLPUSHER_MAX_VELOCITY =3200;
    public final double INTAKE_MAX_VELOCITY =3200;

    public final double DRIVE_SPEED_SLOW = 0.6;
    public final double DRIVE_SPEED_FAST = 0.6;

    public final double TURN_SPEED = 0.3;

    public final double DIST_ROW1=18;
    public final double DIST_ROW2=42;
    
    // Drive distances
    public final double INITIAL_FORWARD_DISTANCE = 12;
    public final double COLLECTION_DISTANCE = 46;
    public final double COLLECTION_DISTANCE_ROW2 = 40;
    public final double PARKING_DISTANCE = 18;
    
    // Turn angles
    public final double INITIAL_TURN_ANGLE = 35;
    public final double TURN_TO_COLLECT_ABSOLUTE = (90);
    public final double TURN_TO_COLLECT = (TURN_TO_COLLECT_ABSOLUTE-INITIAL_TURN_ANGLE);

    
    // Timeout values
    public final double DRIVE_TIMEOUT_SHORT = 10.0;
    public final double DRIVE_TIMEOUT_LONG = 20.0;
    public final double TURN_TIMEOUT = 8.0;
    public final double STRAFE_TIMEOUT = 10.0;
    public final double SLEEP_TIME = 2000;
}


