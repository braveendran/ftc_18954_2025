package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueFar", group = "Autonomous")
public class AutonomousBlueFar extends LinearOpMode {

    // ---------------- HARDWARE DECLARATION (Copied from TeleOp) ----------------
    DcMotor leftFront, rightFront, leftBack, rightBack,launcherMotor;
    DcMotorEx ballPusherMotor, intakeMotor;
    Servo stopperServo;

    // ---------------- LAUNCHER SETTINGS ----------------
    // ---------------- LAUNCHER SETTINGS ----------------
    private final double LAUNCHER_MAX_VELOCITY = 1.0;

    private final double LAUNCHER_POS1_VELOCITY = LAUNCHER_MAX_VELOCITY*.95;

    private final double LAUNCHER_POS2_VELOCITY = LAUNCHER_MAX_VELOCITY * .8;

    private final double BALLPUSHER_MAX_VELOCITY =3200;

    private final double INTAKE_MAX_VELOCITY =3200;

    // ---------------- STOPPER SETTINGS ----------------
    private final double STOPPER_CLOSED = 0.70;
    private final double STOPPER_OPEN = .94;

    // ---------------- AUTONOMOUS CONSTANTS ----------------
    // These values MUST be tuned for your specific robot!
    static final double COUNTS_PER_MOTOR_REV = 537.7; // Example for a goBILDA 5203 series motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Speeds for autonomous movements
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    // Enum for parking positions
    private enum ParkingZone {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private ParkingZone detectedZone = ParkingZone.MIDDLE; // Default if vision fails

    @Override
    public void runOpMode() {

        // ---------------- INIT & HARDWARE MAPPING ----------------
        initializeHardware();

        // ---------------- VISION INITIALIZATION (Placeholder) ----------------
        // In a real robot, you would initialize your camera and TensorFlow/AprilTag pipeline here.
        // This function simulates detecting a signal cone.
        //detectSignalCone();

        // This command waits for the driver to press the START button.
        waitForStart();

        // ---------------- AUTONOMOUS SEQUENCE STARTS HERE ----------------

        // Step 1: Move forward away from the wall to get clearance for shooting.
        encoderDrive(DRIVE_SPEED/2, 12, 12, 5.0*2); // Move forward 30 inches

        // Step 2: Turn towards the high goal. From the far side, this might be a 45-degree turn.
        // A positive angle turns left.
        turn(TURN_SPEED/2, 35, 4.0*2);

        // Step 3: Shoot one Power Core into the high goal.
        shootPowerCore(LAUNCHER_POS1_VELOCITY,false);

        turn(TURN_SPEED/2, -35, 4.0*2);

        encoderDrive(DRIVE_SPEED/2, 18, 18, 5.0*2);

        turn(TURN_SPEED/2, 135, 4.0*4);

        TurnOnIntake();

        encoderDrive(DRIVE_SPEED/2, 46, 46, 10*2);

        TurnOffIntake();

        encoderDrive(DRIVE_SPEED/2, -46, -46, 5.0*2);

        turn(TURN_SPEED/2, -135, 4.0*2);

        encoderDrive(DRIVE_SPEED/2, -20, -20, 5.0*2);

        turn(TURN_SPEED/2, 30, 4.0*2);

        shootPowerCore(LAUNCHER_POS1_VELOCITY,false);

        //Move out of the shooting
        encoderDrive(DRIVE_SPEED/2, 18, 18, 5.0*2);

        // Step 4: Turn back to be parallel with the field walls.
       // turn(TURN_SPEED, -45, 4.0);



        //encoderDrive(DRIVE_SPEED, 18, 18, 5.0);
        // Step 5: Park in the zone determined by the vision system.
//        switch (detectedZone) {
//            case LEFT:
//                telemetry.addData("Parking", "Zone LEFT");
//                telemetry.update();
//                strafe(DRIVE_SPEED, -24, 5.0); // Strafe left 24 inches
//                break;
//            case RIGHT:
//                telemetry.addData("Parking", "Zone RIGHT");
//                telemetry.update();
//                strafe(DRIVE_SPEED, 24, 5.0); // Strafe right 24 inches
//                break;
//            case MIDDLE:
//                // No strafe needed for the middle zone, just drive forward slightly
//                telemetry.addData("Parking", "Zone MIDDLE");
//                telemetry.update();
//                encoderDrive(DRIVE_SPEED, 6, 6, 2.0); // Move forward 6 inches
//                break;
//        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
        sleep(2000); // Keep telemetry on screen for 2 seconds
    }

    /**
     * Initializes all hardware components, sets directions, and zero power behaviors.
     */
    public void initializeHardware() {
        // Hardware mapping
        leftFront = hardwareMap.dcMotor.get("FrontLeft");
        rightFront = hardwareMap.dcMotor.get("FrontRight");
        leftBack = hardwareMap.dcMotor.get("BackLeft");
        rightBack = hardwareMap.dcMotor.get("BackRight");
        ballPusherMotor = hardwareMap.get(DcMotorEx.class, "ballPusherMotor");
        launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        stopperServo = hardwareMap.get(Servo.class, "stopperServo");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        ballPusherMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set motor modes
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ballPusherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stopper initial position
        stopperServo.setPosition(STOPPER_CLOSED);

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();
    }

    /**
     * Placeholder method for vision detection.
     * In a real competition, this would use TensorFlow, AprilTags, or color sensors.
     */
    public void detectSignalCone() {
        telemetry.addData("Vision", "Detecting Signal...");
        telemetry.update();
        // ** THIS IS A PLACEHOLDER **
        // Replace this with your actual vision detection logic.
        // For testing, you can manually set it:
        // detectedZone = ParkingZone.LEFT;
        // detectedZone = ParkingZone.RIGHT;
        detectedZone = ParkingZone.MIDDLE;

        sleep(1000); // Simulate time taken for detection
        telemetry.addData("Vision", "Detected Zone: " + detectedZone.toString());
        telemetry.update();
    }


    public void StartShooter(double LauncherPower) {
        launcherMotor.setPower(LauncherPower);
        ballPusherMotor.setVelocity(BALLPUSHER_MAX_VELOCITY);
    }
    /**
     * A self-contained function to shoot 3 Power Core.
     */
    public void shootPowerCore(double LauncherPower,boolean started) {
        telemetry.addData("Shooter", "Starting sequence...");
        telemetry.update();

        // Spin up the launcher motor
        if(!started) {
            launcherMotor.setPower(LauncherPower);

            ballPusherMotor.setVelocity(BALLPUSHER_MAX_VELOCITY);

            sleep(1200); // Wait 1.2 seconds for the motor to reach full speed
        }

        for (int i=0;i<4;i++) {
            // Open the stopper to feed the Power Core
            stopperServo.setPosition(STOPPER_OPEN);
            sleep(500); // Wait 0.5 seconds for the core to pass

            // Close the stopper and turn off the launcher
            stopperServo.setPosition(STOPPER_CLOSED);
            sleep(500);
        }

        launcherMotor.setPower(0);
        ballPusherMotor.setVelocity(0);

        telemetry.addData("Shooter", "Sequence complete.");
        telemetry.update();
    }

    public void TurnOnIntake()
    {
        intakeMotor.setVelocity(INTAKE_MAX_VELOCITY);
        ballPusherMotor.setVelocity(BALLPUSHER_MAX_VELOCITY);
    }

    public void TurnOffIntake()
    {
        intakeMotor.setVelocity(0);
        ballPusherMotor.setVelocity(0);
    }

    /**
     * Sets the RunMode for all four drive motors.
     */
    public void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    /**
     * Method to perform a relative move, based on encoder counts.
     * @param speed The speed to drive at (-1.0 to 1.0)
     * @param leftInches The distance the left wheels should travel in inches
     * @param rightInches The distance the right wheels should travel in inches
     * @param timeoutS The maximum time this movement is allowed to take
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        if (!opModeIsActive()) return;

        int leftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        rightBack.setTargetPosition(rightBackTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        leftFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            // Display telemetry
            telemetry.addData("Path", "Running to L:%7d R:%7d", leftFrontTarget, rightFrontTarget);
            telemetry.addData("Current", "Running at L:%7d R:%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Turn off RUN_TO_POSITION
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Method to strafe left or right. A positive inches value strafes right.
     */
    public void strafe(double speed, double inches, double timeoutS) {
        // For strafing, the motor directions are:
        //Right: -LF, +LB, +RF, -RB
        // Left:  +LF, -LB, -RF, +RB
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);


    }

    /**
     * Method to turn the robot. A positive angle turns left.
     */
    public void turn(double speed, double angle, double timeoutS) {
        // This is a simplified turn. For more accuracy, use the IMU (Gyro).
        // The distance each wheel travels is based on the turning radius.
        double turnRadiusInches = 9.0; // Half the distance between the wheels
        double turnDistanceInches = (angle / 360.0) * (2 * 3.1415 * turnRadiusInches);
        encoderDrive(speed, -turnDistanceInches, turnDistanceInches, timeoutS);
    }

    /**
     * Overloaded encoderDrive for mecanum strafing.
     */
    public void encoderDrive(double speed, double lfInches, double rfInches, double lbInches, double rbInches, double timeoutS) {
        if (!opModeIsActive()) return;



        int lfTarget = leftFront.getCurrentPosition() + (int)(lfInches * COUNTS_PER_INCH);
        int rfTarget = rightFront.getCurrentPosition() + (int)(rfInches * COUNTS_PER_INCH);
        int lbTarget = leftBack.getCurrentPosition() + (int)(lbInches * COUNTS_PER_INCH);
        int rbTarget = rightBack.getCurrentPosition() + (int)(rbInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(lfTarget);
        rightFront.setTargetPosition(rfTarget);
        leftBack.setTargetPosition(lbTarget);
        rightBack.setTargetPosition(rbTarget);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Strafing...");
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}