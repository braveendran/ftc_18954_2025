package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name = "RedClose", group = "Autonomous")
public class AutonRedClose extends LinearOpMode {

    private final double LAUNCHER_MAX_POWER = 1.0;
    private final double LAUNCHER_POS1_POWER = LAUNCHER_MAX_POWER*.75;


    private final double BALLPUSHER_MAX_VELOCITY =3200;
    private final double INTAKE_MAX_VELOCITY =3200;

    static final double DRIVE_SPEED_SLOW = 0.3;
    static final double DRIVE_SPEED_FAST = 0.6;

    static final double TURN_SPEED = 0.5;




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
        objCommonFunc.StartShooter(LAUNCHER_POS1_POWER,BALLPUSHER_MAX_VELOCITY );


        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, -30, -30 , 5.0*2); // Move forward 30 inches

        objCommonFunc.shootPowerCore(LAUNCHER_POS1_POWER,true,BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.strafe(DRIVE_SPEED_SLOW,48, 10);

        objCommonFunc.TurnOnIntake(INTAKE_MAX_VELOCITY,BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, 42, 42, 10*2);

        objCommonFunc.TurnOffIntake();

        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, -42, -42, 5.0*2);

        objCommonFunc.StartShooter(LAUNCHER_POS1_POWER,BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.strafe(DRIVE_SPEED_SLOW,-48, 10);

        objCommonFunc.shootPowerCore(LAUNCHER_POS1_POWER,true,BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.strafe(DRIVE_SPEED_SLOW,45, -10);


        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        sleep(2000); // Keep telemetry on screen for 2 seconds
    }




}