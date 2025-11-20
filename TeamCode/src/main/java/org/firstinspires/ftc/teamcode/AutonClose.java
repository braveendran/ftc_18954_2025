package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonClose {
    
    public enum Alliance {
        BLUE, RED
    }
    
    private LinearOpMode opMode;
    private AutonCloseParams params;
    private CommonFunc_18954 objCommonFunc;
    private Alliance alliance;
    private boolean twoRowMode;
    
    public AutonClose(LinearOpMode opMode, Alliance alliance) {
        this(opMode, alliance, false);
    }
    
    public AutonClose(LinearOpMode opMode, Alliance alliance, boolean twoRowMode) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.twoRowMode = twoRowMode;
        this.params = new AutonCloseParams();
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
        objCommonFunc.StartShooter(params.LAUNCHER_POS1_RPM, params.BALLPUSHER_MAX_VELOCITY);
        
        // Move backward from starting position
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -params.DRIVE_BACK_DISTANCE, -params.DRIVE_BACK_DISTANCE, params.DRIVE_TIMEOUT);
        
        // First strafe and turn to shooting position
        if (alliance == Alliance.BLUE) {
            objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.STRAFE_DISTANCE_1, params.STRAFE_TIMEOUT);
            objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.STRAFE_DISTANCE_1, params.STRAFE_TIMEOUT);
            objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
        }
        
        // First shooting sequence
        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM, true, params.BALLPUSHER_MAX_VELOCITY);
        
        // Turn back and strafe to collection position
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
            objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.STRAFE_DISTANCE_2, params.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
            objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW1_MOVE, params.STRAFE_TIMEOUT);
        }
        
        // Turn on intake and collect game elements
        objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY, params.BALLPUSHER_MAX_VELOCITY);
        
        // Drive forward to collection area
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, params.DRIVE_FORWARD_DISTANCE, params.DRIVE_FORWARD_DISTANCE, params.DRIVE_TIMEOUT);
        
        // Drive back from collection area
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -params.DRIVE_FORWARD_DISTANCE, -params.DRIVE_FORWARD_DISTANCE, params.DRIVE_TIMEOUT);
        
        // Strafe to second shooting position
        if (alliance == Alliance.BLUE) {
            objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.STRAFE_DISTANCE_2, params.STRAFE_TIMEOUT);
            objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW1_MOVE, params.STRAFE_TIMEOUT);
            objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
        }
        
        // Second shooting sequence
        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM, true, params.BALLPUSHER_MAX_VELOCITY);
        
        // Turn back and turn off intake
        if (alliance == Alliance.BLUE) {
            objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
        }
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Strafe to second row collection position
            if (alliance == Alliance.BLUE) {
                objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW2_MOVE, params.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW2_MOVE, params.STRAFE_TIMEOUT);
            }
            
            // Turn on intake and collect from second row
            objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY, params.BALLPUSHER_MAX_VELOCITY);
            
            // Drive forward to second row collection area
            objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, params.DRIVE_FORWARD_DISTANCE_ROW2, params.DRIVE_FORWARD_DISTANCE_ROW2, params.DRIVE_TIMEOUT);
            
            // Drive back from second row collection area
            objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -params.DRIVE_FORWARD_DISTANCE_ROW2, -params.DRIVE_FORWARD_DISTANCE_ROW2, params.DRIVE_TIMEOUT);
            
            // Strafe to third shooting position
            if (alliance == Alliance.BLUE) {
                objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW2_MOVE, params.STRAFE_TIMEOUT);
                objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW2_MOVE, params.STRAFE_TIMEOUT);
                objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
            }
            
            // Third shooting sequence
            objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM, true, params.BALLPUSHER_MAX_VELOCITY);
            
            // Turn back after third shooting
            if (alliance == Alliance.BLUE) {
                objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, params.TURN_TIMEOUT);
            }
        }
        
        objCommonFunc.TurnOffIntake();
        
        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        opMode.sleep((int)params.SLEEP_TIME);
    }
}
