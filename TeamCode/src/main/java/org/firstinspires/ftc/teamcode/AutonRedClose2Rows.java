package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name = "RedClose2Rows", group = "Autonomous")

public class AutonRedClose2Rows extends LinearOpMode {



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

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -45, -45 , 5.0*2); // Move forward 30 inches

        //objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW,20, 10);
        //objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, 4.0*2);

        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,true,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, 4.0*2);

        objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW,12, 10);

        objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, 43, 43, 10*2);

        //objCommonFunc.TurnOffIntake();

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -42, -42, 5.0*2);

        //objCommonFunc.StartShooter(params.LAUNCHER_POS1_POWER,params.BALLPUSHER_MAX_VELOCITY);


        //objCommonFunc.strafe_left(params.DRIVE_SPEED_SLOW,params.DIST_ROW1_MOVE, 10);

        objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING+5, 4.0*2);

        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,true,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.turn(params.TURN_SPEED, -params.ANGLE_TO_TURN_SHOOTING, 4.0*2);
		
		//row 2 shooting
        objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW,params.DIST_ROW2_MOVE, 10);

        objCommonFunc.TurnOnIntake(params.INTAKE_MAX_VELOCITY,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, 42, 42, 10*2);

        //objCommonFunc.TurnOffIntake();

        objCommonFunc.encoderDrive(params.DRIVE_SPEED_SLOW, -42, -40, 5.0*2);

        //objCommonFunc.StartShooter(params.LAUNCHER_POS1_POWER,params.BALLPUSHER_MAX_VELOCITY);


        objCommonFunc.strafe_left(1.0,params.DIST_ROW2_MOVE, 10);

        objCommonFunc.turn(params.TURN_SPEED, params.ANGLE_TO_TURN_SHOOTING, 4.0*2);

        objCommonFunc.shootPowerCore(params.LAUNCHER_POS1_RPM,true,params.BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.turn(params.TURN_SPEED, -(params.ANGLE_TO_TURN_SHOOTING+10), 4.0*2);

        objCommonFunc.TurnOffIntake();

        //objCommonFunc.strafe_right(params.DRIVE_SPEED_SLOW,10, 10);


        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        sleep(2000); // Keep telemetry on screen for 2 seconds
    }




}