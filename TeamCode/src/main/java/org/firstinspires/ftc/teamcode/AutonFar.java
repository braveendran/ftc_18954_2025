package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonFar {
    
    public enum Alliance {
        BLUE, RED
    }
    
    private LinearOpMode opMode;
    private AutonFarParams params;
    private CommonFunc_18954 objCommonFunc;
    private Alliance alliance;
    private boolean twoRowMode;
    
    public AutonFar(LinearOpMode opMode, Alliance alliance) {
        this(opMode, alliance, false);
    }
    
    public AutonFar(LinearOpMode opMode, Alliance alliance, boolean twoRowMode) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.twoRowMode = twoRowMode;
        this.params = new AutonFarParams();
        this.objCommonFunc = new CommonFunc_18954(opMode);
    }
    
    public void runAutonomousSequence() {
        // ---------------- INIT & HARDWARE MAPPING ----------------
        objCommonFunc.initializeHardware();
        
        // ---------------- VISION INITIALIZATION (Placeholder) ----------------
        // In a real robot, you would initialize your camera and TensorFlow/AprilTag pipeline here.
        // This function simulates detecting a signal cone.
        //detectSignalCone();
        
        // This command waits for the driver to press the START button.
        opMode.waitForStart();
        
        // ---------------- AUTONOMOUS SEQUENCE STARTS HERE ----------------
        
        // Step 1: Move forward away from the wall to get clearance for shooting
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, params.INITIAL_FORWARD_DISTANCE, params.INITIAL_FORWARD_DISTANCE, params.DRIVE_TIMEOUT_SHORT);
        
        // Step 2: Turn towards the high goal
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(params.TURN_SPEED, params.INITIAL_TURN_ANGLE, params.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(params.TURN_SPEED, -params.INITIAL_TURN_ANGLE, params.TURN_TIMEOUT);
        }
        
        // Step 3: Shoot first Power Core into the high goal
        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM, false, params.BALLPUSHER_MAX_VELOCITY);
        
        // Step 4: Turn to collection position
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(params.TURN_SPEED, params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
            objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW1, params.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(params.TURN_SPEED, -params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
            objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW1, params.STRAFE_TIMEOUT);
        }
        
        // Step 5: Turn on intake and collect from first row
        objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY, params.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, params.COLLECTION_DISTANCE, params.COLLECTION_DISTANCE, params.DRIVE_TIMEOUT_LONG);
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -params.COLLECTION_DISTANCE, -params.COLLECTION_DISTANCE, params.DRIVE_TIMEOUT_SHORT);
        
        // Step 6: Return to shooting position for second shot
        if (alliance == Alliance.BLUE) {
            objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW1, params.STRAFE_TIMEOUT);
            objCommonFunc.turn(params.TURN_SPEED, -params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW1, params.STRAFE_TIMEOUT);
            objCommonFunc.turn(params.TURN_SPEED, params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
        }
        
        // Step 7: Second shooting sequence
        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM, false, params.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.TurnOffIntake();
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Step 8: Move to second row collection
            if (alliance == Alliance.BLUE) {
                objCommonFunc.turn(params.TURN_SPEED, params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
                objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW2, params.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(params.TURN_SPEED, -params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
                objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW2, params.STRAFE_TIMEOUT);
            }
            
            // Step 9: Collect from second row
            objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY, params.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, params.COLLECTION_DISTANCE_ROW2, params.COLLECTION_DISTANCE_ROW2, params.DRIVE_TIMEOUT_LONG);
            objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -params.COLLECTION_DISTANCE_ROW2, -params.COLLECTION_DISTANCE_ROW2, params.DRIVE_TIMEOUT_SHORT);
            
            // Step 10: Return to shooting position for third shot
            if (alliance == Alliance.BLUE) {
                objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW2, params.STRAFE_TIMEOUT);
                objCommonFunc.turn(params.TURN_SPEED, -params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW2, params.STRAFE_TIMEOUT);
                objCommonFunc.turn(params.TURN_SPEED, params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
            }
            
            // Step 11: Third shooting sequence
            objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM, false, params.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.TurnOffIntake();
        }
        
        // END OF Auton - Go to parking
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(params.TURN_SPEED, params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(params.TURN_SPEED, -params.TURN_TO_COLLECT, params.TURN_TIMEOUT);
        }
        
        // Move out of the shooting area for parking
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, params.PARKING_DISTANCE, params.PARKING_DISTANCE, params.DRIVE_TIMEOUT_SHORT);
        
        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        opMode.sleep((int)params.SLEEP_TIME);
    }
}
