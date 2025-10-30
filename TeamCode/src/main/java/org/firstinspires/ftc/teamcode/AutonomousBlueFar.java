package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueFar", group = "Autonomous")
public class AutonomousBlueFar extends LinearOpMode {

    private final double LAUNCHER_MAX_POWER = 1.0;
    private final double LAUNCHER_BACKSHOOT_POWER = LAUNCHER_MAX_POWER*.75;


    private final double BALLPUSHER_MAX_VELOCITY =3200;
    private final double INTAKE_MAX_VELOCITY =3200;

    static final double DRIVE_SPEED_SLOW = 0.3;
    static final double DRIVE_SPEED_FAST = 0.6;

    static final double TURN_SPEED = 0.3;

    private CommonFunc_18954 objCommonFunc;

    @Override
    public void runOpMode() {

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
        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, 12, 12, 5.0*2); // Move forward 30 inches

        // Step 2: Turn towards the high goal. From the far side, this might be a 45-degree turn.
        // A positive angle turns left.
        objCommonFunc.turn(TURN_SPEED, 35, 4.0*2);

        // Step 3: Shoot one Power Core into the high goal.
        objCommonFunc.shootPowerCore(LAUNCHER_BACKSHOOT_POWER,false,BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.turn(TURN_SPEED, -35, 4.0*2);

        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, 18, 18, 5.0*2);

        objCommonFunc.turn(TURN_SPEED, 135, 4.0*4);

        objCommonFunc.TurnOnIntake(INTAKE_MAX_VELOCITY,BALLPUSHER_MAX_VELOCITY);

        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, 46, 46, 10*2);

        objCommonFunc.TurnOffIntake();

        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, -46, -46, 5.0*2);

        objCommonFunc.turn(TURN_SPEED, -135, 4.0*2);

        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, -20, -20, 5.0*2);

        objCommonFunc.turn(TURN_SPEED, 30, 4.0*2);

        objCommonFunc.shootPowerCore(LAUNCHER_BACKSHOOT_POWER,false,BALLPUSHER_MAX_VELOCITY);

        //Move out of the shooting
        objCommonFunc.encoderDrive(DRIVE_SPEED_SLOW, 18, 18, 5.0*2);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        sleep(2000); // Keep telemetry on screen for 2 seconds
    }

}
