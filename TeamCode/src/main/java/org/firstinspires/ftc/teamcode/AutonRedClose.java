package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name = "RedClose", group = "Autonomous")
public class AutonRedClose extends LinearOpMode {



    private AutonCloseParams params = new AutonCloseParams();
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
        //objCommonFunc.StartShooter(params.LAUNCHER_POS1_POWER,params.BALLPUSHER_MAX_VELOCITY );
        //objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY,params.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.StartShooter(params.LAUNCHER_POS1_RPM,params.BALLPUSHER_MAX_VELOCITY );

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -params.DRIVE_BACK_DISTANCE, -params.DRIVE_BACK_DISTANCE, params.DRIVE_TIMEOUT); // Move backward 30 inches

        objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.STRAFE_DISTANCE_1, params.STRAFE_TIMEOUT);
        objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, 4.0*2);

        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,true,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, 4.0*2);

        objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW, params.DIST_ROW1_MOVE, params.STRAFE_TIMEOUT);

        objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, params.DRIVE_FORWARD_DISTANCE, params.DRIVE_FORWARD_DISTANCE, params.DRIVE_TIMEOUT);

        //objCommonFunc.TurnOffIntake();

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -params.DRIVE_FORWARD_DISTANCE, -params.DRIVE_FORWARD_DISTANCE, params.DRIVE_TIMEOUT);

        //objCommonFunc.StartShooter(params.LAUNCHER_POS1_POWER,params.BALLPUSHER_MAX_VELOCITY);


        objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW, params.DIST_ROW1_MOVE, params.STRAFE_TIMEOUT);

        objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, 4.0*2);

        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,true,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, 4.0*2);

        objCommonFunc.TurnOffIntake();

        //objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW,10, 10);


        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        sleep((int)params.SLEEP_TIME); // Keep telemetry on screen for 2 seconds
    }




}