package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.logic.CommonDefs;
import org.firstinspires.ftc.teamcode.logic.CommonFunc_18954;
import org.firstinspires.ftc.teamcode.params.AutonCloseParams;
import org.firstinspires.ftc.teamcode.params.AutonFarParams;
import org.firstinspires.ftc.teamcode.logic.LimeLightHandler;
import org.firstinspires.ftc.teamcode.logic.LocalizerDecode;
import org.firstinspires.ftc.teamcode.logic.DriverIndicationLED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutonMovement {
    
    private LinearOpMode opMode;
    private AutonCloseParams closeParams;
    private AutonFarParams farParams;
    private CommonFunc_18954 objCommonFunc;
    private CommonDefs.Alliance alliance;
    private CommonDefs.PositionType positionType;
    private boolean twoRowMode;
    private LocalizerDecode mLocalizer;
    private LimeLightHandler mLimeLightHandler;
    private DriverIndicationLED mDriverIndicationLED;
    private IMU imu;
    LLResult CameraResult;




    
    // Constructor for Close positioning
//    public AutonMovement(LinearOpMode opMode, CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
//        this(opMode, alliance, positionType, false);
//    }
    
    // Constructor with two-row mode option
    public AutonMovement(LinearOpMode opMode, CommonDefs.Alliance alliance, CommonDefs.PositionType positionType, boolean twoRowMode ) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.positionType = positionType;
        this.twoRowMode = twoRowMode;
        this.closeParams = new AutonCloseParams();
        this.farParams = new AutonFarParams();
        this.objCommonFunc = new CommonFunc_18954(opMode, alliance, this::PeriodicUpdate);
        objCommonFunc.initializeHardware();

        mDriverIndicationLED = new DriverIndicationLED(opMode.hardwareMap);
        // Note: IMU and other hardware will be initialized when initializeHardware() is called in the sequence methods
        imu = opMode.hardwareMap.get(IMU.class, "imu"); // Get IMU directly from hardware map
        mLimeLightHandler = new LimeLightHandler(imu, opMode.hardwareMap, alliance);
        mLocalizer = new LocalizerDecode(alliance, mLimeLightHandler, mDriverIndicationLED , this.objCommonFunc.Get_Forwardpod(), this.objCommonFunc.Get_StraferPod(), this.objCommonFunc.getIMU() );
    }
    
    public void runAutonomousSequence() {
        if (positionType == CommonDefs.PositionType.CLOSE) {
            runCloseAutonomousSequence();
        } else {
            runFarAutonomousSequence();
        }
        //stop the localizer and the camera
        mLocalizer.Stop();
    }

    private  void PeriodicUpdate(long current_ms) {
        if(opMode.opModeIsActive()) {
            if (mLimeLightHandler != null && mLocalizer != null) {
                CameraResult = mLocalizer.update(current_ms);
            }
        }
    }
    
    private void runCloseAutonomousSequence() {
        double turn_to_shoot_angle=0.0;
        // ---------------- INIT & HARDWARE MAPPING ----------------

        
        // ---------------- VISION INITIALIZATION (Placeholder) ----------------
        // In a real robot, you would initialize your camera and TensorFlow/AprilTag pipeline here.
        // This function simulates detecting a signal cone.
        //detectSignalCone();
        
        // This command waits for the driver to press the START button.
        opMode.waitForStart();
        
        // ---------------- AUTONOMOUS SEQUENCE STARTS HERE ----------------
        opMode.telemetry.addData("Starting IMU", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
        // Step 1: Move forward away from the wall to get clearance for shooting
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, closeParams.INITIAL_BACKWARD_DIST, closeParams.INITIAL_BACKWARD_DIST, closeParams.DRIVE_TIMEOUT_SHORT);
        PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 1", objCommonFunc.getIMUYaw());
        // Step 2: Turn towards the high goal (more conservative angle since we're close)
        //        if (alliance == CommonDefs.Alliance.BLUE) {
        //            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.INITIAL_TURN_ANGLE, closeParams.TURN_TIMEOUT);
        //        } else { // RED
        //            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.INITIAL_TURN_ANGLE, closeParams.TURN_TIMEOUT);
        //        }
        
        // Step 3: Shoot first Power Core into the high goal
        turn_to_shoot_angle=objCommonFunc.turn_to_shoot(closeParams.TURN_SPEED,0,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), closeParams.TURN_TIMEOUT, CameraResult, mLocalizer);
        objCommonFunc.shootPowerCore(closeParams.LAUNCHER_POS1_RPM, false, closeParams.BALLPUSHER_MAX_VELOCITY,false,4);
        PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 3", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 4: Navigate to first row collection position
        if (alliance == CommonDefs.Alliance.RED) {
            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT,closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT,-closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
        }
        PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 4", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 5: Turn on intake and collect from first row
        objCommonFunc.TurnOnIntake(closeParams.INTAKE_MAX_VELOCITY, closeParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_INTAKE, closeParams.COLLECTION_DISTANCE_ROW1, closeParams.COLLECTION_DISTANCE_ROW1, closeParams.DRIVE_TIMEOUT_LONG);
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, -closeParams.COLLECTION_DISTANCE_ROW1, -closeParams.COLLECTION_DISTANCE_ROW1, closeParams.DRIVE_TIMEOUT_SHORT);
        //PeriodicUpdate(System.currentTimeMillis());

        opMode.telemetry.addData("Step 5", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();

        // Step 6: Return to shooting position for second shot
        if (alliance == CommonDefs.Alliance.RED) {
            objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
            //objCommonFunc.turn(closeParams.TURN_SPEED, -(closeParams.TURN_TO_COLLECT + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA),0, closeParams.TURN_TIMEOUT);
            PeriodicUpdate(System.currentTimeMillis());
            turn_to_shoot_angle=objCommonFunc.turn_to_shoot(closeParams.TURN_SPEED,-(closeParams.TURN_TO_COLLECT + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA),0, closeParams.TURN_TIMEOUT, CameraResult, mLocalizer);
     
        } else { // RED
            objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
            //objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT+ closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA,0, closeParams.TURN_TIMEOUT);
            PeriodicUpdate(System.currentTimeMillis());
            turn_to_shoot_angle=objCommonFunc.turn_to_shoot(closeParams.TURN_SPEED,closeParams.TURN_TO_COLLECT+ closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA,0, closeParams.TURN_TIMEOUT, CameraResult, mLocalizer);
     
        }
        //PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 6", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 7: Second shooting sequence
          objCommonFunc.shootPowerCore(closeParams.LAUNCHER_POS1_RPM, false, closeParams.BALLPUSHER_MAX_VELOCITY,false,4);
        objCommonFunc.TurnOffIntake();
        //PeriodicUpdate(System.currentTimeMillis());

        opMode.telemetry.addData("Step 7", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Step 8: Move to second row collection
            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT, closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2 , closeParams.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT, - closeParams.TURN_TO_COLLECT,closeParams.TURN_TIMEOUT);
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2, closeParams.STRAFE_TIMEOUT);
            }
            opMode.telemetry.addData("Step 8", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();
            
            // Step 9: Collect from second row
            objCommonFunc.TurnOnIntake(closeParams.INTAKE_MAX_VELOCITY, closeParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_INTAKE, closeParams.COLLECTION_DISTANCE_ROW2, closeParams.COLLECTION_DISTANCE_ROW2, closeParams.DRIVE_TIMEOUT_LONG);
            objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, -closeParams.COLLECTION_DISTANCE_ROW2, -closeParams.COLLECTION_DISTANCE_ROW2, closeParams.DRIVE_TIMEOUT_SHORT);

            opMode.telemetry.addData("Step 9", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();
            // Step 10: Return to shooting position for third shot
            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_ROW2_HIGHSPEED, closeParams.DIST_ROW2 +closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);
                //objCommonFunc.turn(closeParams.TURN_SPEED, - (closeParams.TURN_TO_SHOOT_ROW2 + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA), closeParams.TURN_TO_SHOOT_ROW2_ABSOLUTE,closeParams.TURN_TIMEOUT);
                PeriodicUpdate(System.currentTimeMillis());
                turn_to_shoot_angle=objCommonFunc.turn_to_shoot(closeParams.TURN_SPEED,- (closeParams.TURN_TO_SHOOT_ROW2 + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA), closeParams.TURN_TO_SHOOT_ROW2_ABSOLUTE, closeParams.TURN_TIMEOUT, CameraResult, mLocalizer);
      
            } else { // RED
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_ROW2_HIGHSPEED, closeParams.DIST_ROW2 +closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);
                //objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_SHOOT_ROW2 + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA, -closeParams.TURN_TO_SHOOT_ROW2_ABSOLUTE,closeParams.TURN_TIMEOUT);
                PeriodicUpdate(System.currentTimeMillis());
                turn_to_shoot_angle=objCommonFunc.turn_to_shoot(closeParams.TURN_SPEED,closeParams.TURN_TO_SHOOT_ROW2 + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA, -closeParams.TURN_TO_SHOOT_ROW2_ABSOLUTE, closeParams.TURN_TIMEOUT, CameraResult, mLocalizer);
      
            }
            opMode.telemetry.addData("Step 10", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();
            // Step 11: Third shooting sequence
          
            //objCommonFunc.shootPowerCore(closeParams.LAUNCHER_POS1_RPM, false, closeParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.TurnOffIntake();
        }
        else {

            // END OF Auton - Go to parking
            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT,closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT,-closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            }

            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);

            } else { // RED
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);

            }
        }
//
//
//

        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        //PeriodicUpdate(System.currentTimeMillis());
        //opMode.sleep((int)closeParams.SLEEP_TIME);
    }
    
    private void runFarAutonomousSequence() {
        double turn_to_shoot_angle=0.0;
        // ---------------- INIT & HARDWARE MAPPING ----------------
        objCommonFunc.initializeHardware();
        
        // ---------------- VISION INITIALIZATION (Placeholder) ----------------
        // In a real robot, you would initialize your camera and TensorFlow/AprilTag pipeline here.
        // This function simulates detecting a signal cone.
        //detectSignalCone();
        
        // This command waits for the driver to press the START button.
        opMode.waitForStart();
        
        // ---------------- AUTONOMOUS SEQUENCE STARTS HERE ----------------

        opMode.telemetry.addData("Starting IMU", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 1: Move forward away from the wall to get clearance for shooting
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, farParams.INITIAL_FORWARD_DISTANCE, farParams.INITIAL_FORWARD_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);
        PeriodicUpdate(System.currentTimeMillis());

        opMode.telemetry.addData("Step 1", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();

        //Step 2: Turn towards the high goal
        if (alliance == CommonDefs.Alliance.BLUE) {
            //objCommonFunc.turn(farParams.TURN_SPEED, farParams.INITIAL_TURN_ANGLE,farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
            PeriodicUpdate(System.currentTimeMillis());
             turn_to_shoot_angle=objCommonFunc.turn_to_shoot(farParams.TURN_SPEED,farParams.INITIAL_TURN_ANGLE,farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT, CameraResult, mLocalizer);
     
        } else { // RED
            //objCommonFunc.turn(farParams.TURN_SPEED, -farParams.INITIAL_TURN_ANGLE,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
            PeriodicUpdate(System.currentTimeMillis());
             turn_to_shoot_angle=objCommonFunc.turn_to_shoot(farParams.TURN_SPEED,-farParams.INITIAL_TURN_ANGLE,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT, CameraResult, mLocalizer);
     
        }

        //PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 2", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();

        // Step 3: Shoot first Power Core into the high goal
        objCommonFunc.shootPowerCore(farParams.LAUNCHER_POS1_RPM, false, farParams.BALLPUSHER_MAX_VELOCITY,true,4);
        //PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 3", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        // Step 4: Turn to collection position
        if (alliance == CommonDefs.Alliance.BLUE) {
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
            objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT,-farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
            objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
        }
        //PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 4", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        // Step 5: Turn on intake and collect from first row
        objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_INTAKE, farParams.COLLECTION_DISTANCE, farParams.COLLECTION_DISTANCE, farParams.DRIVE_TIMEOUT_LONG);
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, -farParams.COLLECTION_DISTANCE, -farParams.COLLECTION_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);
        //PeriodicUpdate(System.currentTimeMillis());

        opMode.telemetry.addData("Step 5", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        // Step 6: Return to shooting position for second shot
        if (alliance == CommonDefs.Alliance.BLUE) {
            objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1 + farParams.DIST_ROW1_ADDITIONAL_RETURN, farParams.STRAFE_TIMEOUT);
            //objCommonFunc.turn(farParams.TURN_SPEED,, farParams.TURN_TIMEOUT);
            PeriodicUpdate(System.currentTimeMillis());
            turn_to_shoot_angle=objCommonFunc.turn_to_shoot(farParams.TURN_SPEED ,-farParams.TURN_TO_COLLECT,farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT, CameraResult, mLocalizer);
    
        } else { // RED
            objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1+ farParams.DIST_ROW1_ADDITIONAL_RETURN, farParams.STRAFE_TIMEOUT);
            //objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
            PeriodicUpdate(System.currentTimeMillis());
            turn_to_shoot_angle=objCommonFunc.turn_to_shoot(farParams.TURN_SPEED,farParams.TURN_TO_COLLECT,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT, CameraResult, mLocalizer);
    
        }
        //PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 6", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 7: Second shooting sequence
         objCommonFunc.shootPowerCore(farParams.LAUNCHER_POS1_RPM, false, farParams.BALLPUSHER_MAX_VELOCITY,true,3);
        objCommonFunc.TurnOffIntake();
        PeriodicUpdate(System.currentTimeMillis());
        opMode.telemetry.addData("Step 7", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Step 8: Move to second row collection
            if (alliance == CommonDefs.Alliance.BLUE) {
                objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
                objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT,-farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
                objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
            }
            opMode.telemetry.addData("Step 8", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();

            
            // Step 9: Collect from second row
            objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_INTAKE, farParams.COLLECTION_DISTANCE_ROW2, farParams.COLLECTION_DISTANCE_ROW2, farParams.DRIVE_TIMEOUT_LONG);
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, -farParams.COLLECTION_DISTANCE_ROW2, -farParams.COLLECTION_DISTANCE_ROW2, farParams.DRIVE_TIMEOUT_SHORT);

            opMode.telemetry.addData("Step 9", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();

//            // Step 10: Return to shooting position for third shot
//            if (alliance == CommonDefs.Alliance.BLUE) {
//                objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
//                objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT,farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
//            } else { // RED
//                objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
//                objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
//            }
//
//            opMode.telemetry.addData("Step 10", objCommonFunc.getIMUYaw());
//            opMode.telemetry.update();
//
//            // Step 11: Third shooting sequence
//            objCommonFunc.shootPowerCore(farParams.LAUNCHER_POS1_RPM, false, farParams.BALLPUSHER_MAX_VELOCITY);
//            objCommonFunc.TurnOffIntake();
        }
        else {

            // END OF Auton - Go to parking
//            if (alliance == CommonDefs.Alliance.BLUE) {
//                objCommonFunc.turn(farParams.TURN_SPEED + .4, farParams.TURN_TO_COLLECT, farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
//            } else { // RED
//                objCommonFunc.turn(farParams.TURN_SPEED + .4, -farParams.TURN_TO_COLLECT, -farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
//            }

            // Move out of the shooting area for parking
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW +.4, farParams.PARKING_DISTANCE, farParams.PARKING_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);
        }
        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        //PeriodicUpdate(System.currentTimeMillis());
        //opMode.sleep((int)farParams.SLEEP_TIME);
    }
}