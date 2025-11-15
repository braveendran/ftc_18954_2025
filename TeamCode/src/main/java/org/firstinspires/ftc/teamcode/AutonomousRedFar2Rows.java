package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedFar2Rows", group = "Autonomous")
public class AutonomousRedFar2Rows extends LinearOpMode {


    // Import parameters from AutonFarParams
    private AutonFarParams params = new AutonFarParams();
    private CommonFunc_18954 objCommonFunc;

    @Override
    public void runOpMode() {

        objCommonFunc = new CommonFunc_18954(this);

        // ---------------- INIT & HARDWARE MAPPING ----------------
        objCommonFunc.initializeHardware();

        // ---------------- VISION INITIALIZATION (Placeholder) ----------------
        // In a real robot, you would initialize your camera and TensorFlow/AprilTag pipeline here.
        // This function simulates detecting a signal cone.
        //detectSignalCone();

        // This command waits for the driver to press the START button.
        waitForStart();

        // ---------------- AUTONOMOUS SEQUENCE STARTS HERE ----------------

        // Step 1: Move forward away from the wall to get clearance for shooting.
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, 12, 12, 5.0*2); // Move forward 30 inches

        // Step 2: Turn towards the high goal. From the far side, this might be a 45-degree turn.
        // A positive angle turns left.
        objCommonFunc.turn(params.TURN_SPEED, -35, 4.0*2);

        // Step 3: Shoot one Power Core into the high goal.
        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,false,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.turn(params.TURN_SPEED, -90, 4.0*2);

        objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW1,  5.0*2);
        
        objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, 46, 46, 10*2);


        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -46, -46, 5.0*2);

        objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW1, 5.0*2);

        objCommonFunc.turn(params.TURN_SPEED, 90, 4.0*2);

        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,false,params.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.TurnOffIntake();

        //Row 2

        objCommonFunc.turn(params.TURN_SPEED, -90, 4.0*2);

        objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW2,  5.0*2);

        objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, 46, 46, 10*2);


        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -46, -46, 5.0*2);

        objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW2, 5.0*2);

        objCommonFunc.turn(params.TURN_SPEED, 90, 4.0*2);

        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,false,params.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.TurnOffIntake();


        //END OF Auton. Go to parking

        objCommonFunc.turn(params.TURN_SPEED, -90, 4.0*2);
        //Move out of the shooting
        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, 18, 18, 5.0*2);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        sleep(2000); // Keep telemetry on screen for 2 seconds
    }

}
