package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonMovement {
    
    public enum Alliance {
        BLUE, RED
    }
    
    public enum PositionType {
        CLOSE, FAR
    }
    
    private LinearOpMode opMode;
    private AutonCloseParams closeParams;
    private AutonFarParams farParams;
    private CommonFunc_18954 objCommonFunc;
    private Alliance alliance;
    private PositionType positionType;
    private boolean twoRowMode;
    
    // Constructor for Close positioning
    public AutonMovement(LinearOpMode opMode, Alliance alliance, PositionType positionType) {
        this(opMode, alliance, positionType, false);
    }
    
    // Constructor with two-row mode option
    public AutonMovement(LinearOpMode opMode, Alliance alliance, PositionType positionType, boolean twoRowMode) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.positionType = positionType;
        this.twoRowMode = twoRowMode;
        this.closeParams = new AutonCloseParams();
        this.farParams = new AutonFarParams();
        this.objCommonFunc = new CommonFunc_18954(opMode);
    }
    
    public void runAutonomousSequence() {
        if (positionType == PositionType.CLOSE) {
            runCloseAutonomousSequence();
        } else {
            runFarAutonomousSequence();
        }
    }
    
    private void runCloseAutonomousSequence() {
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
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, closeParams.INITIAL_FORWARD_DISTANCE, closeParams.INITIAL_FORWARD_DISTANCE, closeParams.DRIVE_TIMEOUT_SHORT);
        
        // Step 2: Turn towards the high goal (more conservative angle since we're close)
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.INITIAL_TURN_ANGLE, closeParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.INITIAL_TURN_ANGLE, closeParams.TURN_TIMEOUT);
        }
        
        // Step 3: Shoot first Power Core into the high goal  
        objCommonFunc.shootPowerCore(closeParams.LAUNCHER_POS1_RPM, false, closeParams.BALLPUSHER_MAX_VELOCITY);
        
        // Step 4: Navigate to first row collection position
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
        }
        
        // Step 5: Turn on intake and collect from first row
        objCommonFunc.TurnOnIntake(closeParams.INTAKE_MAX_VELOCITY, closeParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, closeParams.COLLECTION_DISTANCE, closeParams.COLLECTION_DISTANCE, closeParams.DRIVE_TIMEOUT_LONG);
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, -closeParams.COLLECTION_DISTANCE, -closeParams.COLLECTION_DISTANCE, closeParams.DRIVE_TIMEOUT_SHORT);
        
        // Step 6: Return to shooting position for second shot
        if (alliance == Alliance.BLUE) {
            objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
        }
        
        // Step 7: Second shooting sequence
        objCommonFunc.shootPowerCore(closeParams.LAUNCHER_POS1_RPM, false, closeParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.TurnOffIntake();
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Step 8: Move to second row collection
            if (alliance == Alliance.BLUE) {
                objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2, closeParams.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2, closeParams.STRAFE_TIMEOUT);
            }
            
            // Step 9: Collect from second row
            objCommonFunc.TurnOnIntake(closeParams.INTAKE_MAX_VELOCITY, closeParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, closeParams.COLLECTION_DISTANCE_ROW2, closeParams.COLLECTION_DISTANCE_ROW2, closeParams.DRIVE_TIMEOUT_LONG);
            objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, -closeParams.COLLECTION_DISTANCE_ROW2, -closeParams.COLLECTION_DISTANCE_ROW2, closeParams.DRIVE_TIMEOUT_SHORT);
            
            // Step 10: Return to shooting position for third shot
            if (alliance == Alliance.BLUE) {
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2, closeParams.STRAFE_TIMEOUT);
                objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2, closeParams.STRAFE_TIMEOUT);
                objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            }
            
            // Step 11: Third shooting sequence
            objCommonFunc.shootPowerCore(closeParams.LAUNCHER_POS1_RPM, false, closeParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.TurnOffIntake();
        }
        
        // END OF Auton - Go to parking
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
        }
        
        // Move out of the shooting area for parking
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, closeParams.PARKING_DISTANCE, closeParams.PARKING_DISTANCE, closeParams.DRIVE_TIMEOUT_SHORT);
        
        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        opMode.sleep((int)closeParams.SLEEP_TIME);
    }
    
    private void runFarAutonomousSequence() {
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
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, farParams.INITIAL_FORWARD_DISTANCE, farParams.INITIAL_FORWARD_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);
        
        // Step 2: Turn towards the high goal
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
        }
        
        // Step 3: Shoot first Power Core into the high goal
        objCommonFunc.shootPowerCore(farParams.LAUNCHER_POS1_RPM, false, farParams.BALLPUSHER_MAX_VELOCITY);
        
        // Step 4: Turn to collection position
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
            objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
            objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
        }
        
        // Step 5: Turn on intake and collect from first row
        objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, farParams.COLLECTION_DISTANCE, farParams.COLLECTION_DISTANCE, farParams.DRIVE_TIMEOUT_LONG);
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, -farParams.COLLECTION_DISTANCE, -farParams.COLLECTION_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);
        
        // Step 6: Return to shooting position for second shot
        if (alliance == Alliance.BLUE) {
            objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
        }
        
        // Step 7: Second shooting sequence
        objCommonFunc.shootPowerCore(farParams.LAUNCHER_POS1_RPM, false, farParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.TurnOffIntake();
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Step 8: Move to second row collection
            if (alliance == Alliance.BLUE) {
                objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
                objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
                objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
            }
            
            // Step 9: Collect from second row
            objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, farParams.COLLECTION_DISTANCE_ROW2, farParams.COLLECTION_DISTANCE_ROW2, farParams.DRIVE_TIMEOUT_LONG);
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, -farParams.COLLECTION_DISTANCE_ROW2, -farParams.COLLECTION_DISTANCE_ROW2, farParams.DRIVE_TIMEOUT_SHORT);
            
            // Step 10: Return to shooting position for third shot
            if (alliance == Alliance.BLUE) {
                objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
                objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
                objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
            }
            
            // Step 11: Third shooting sequence
            objCommonFunc.shootPowerCore(farParams.LAUNCHER_POS1_RPM, false, farParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.TurnOffIntake();
        }
        
        // END OF Auton - Go to parking
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT, farParams.TURN_TIMEOUT);
        }
        
        // Move out of the shooting area for parking
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, farParams.PARKING_DISTANCE, farParams.PARKING_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);
        
        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        opMode.sleep((int)farParams.SLEEP_TIME);
    }
}