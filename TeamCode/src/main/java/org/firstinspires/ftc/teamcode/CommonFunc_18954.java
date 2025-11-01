package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CommonFunc_18954 {


    private LinearOpMode opMode; // Store the OpMode reference
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    DcMotor leftFront, rightFront, leftBack, rightBack,launcherMotor;
    DcMotorEx ballPusherMotor, intakeMotor;
    Servo stopperServo;

    boolean bShooterRunning=false;

    static final double COUNTS_PER_MOTOR_REV = 537.7; // Example for a goBILDA 5203 series motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);



    long INITIAL_SPIN_UP_TIME=1300;
    long GATE_DOWN_TIME=500;
    long GATE_UP_TIME=800;

    long MINIMAL_SLEEP_TIME=1;


    private final double STOPPER_CLOSED = 0.70;
    private final double STOPPER_OPEN = .94;






    public CommonFunc_18954(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }


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

    public void StartShooter(double LauncherPower, double BallPusherVelocity) {
        launcherMotor.setPower(LauncherPower);
        ballPusherMotor.setVelocity(3200);
        bShooterRunning=true;
    }
    /**
     * A self-contained function to shoot 3 Power Core.
     */
    public void shootPowerCore(double LauncherPower,boolean unused ,double BallPusherVelocity) {
        //telemetry.addData("Shooter", "Starting sequence...");
        //telemetry.update();

        // Spin up the launcher motor
        if(!bShooterRunning) {
            launcherMotor.setPower(LauncherPower);

            ballPusherMotor.setVelocity(3200);

            opMode.sleep(INITIAL_SPIN_UP_TIME); // Wait 1.2 seconds for the motor to reach full speed
        }

        for (int i=0;i<4;i++) {
            // Open the stopper to feed the Power Core
            stopperServo.setPosition(STOPPER_OPEN);
            opMode.sleep(GATE_DOWN_TIME); // Wait 0.5 seconds for the core to pass

            // Close the stopper and turn off the launcher
            stopperServo.setPosition(STOPPER_CLOSED);
            opMode.sleep(GATE_UP_TIME);
        }

        launcherMotor.setPower(0);
        ballPusherMotor.setVelocity(0);

        bShooterRunning=false;

    }

    public void TurnOnIntake(double IntakeVelocity,double BallPusherVelocity)
    {
        intakeMotor.setVelocity(IntakeVelocity);
        ballPusherMotor.setVelocity(BallPusherVelocity);
    }

    public void TurnOffIntake()
    {
        intakeMotor.setVelocity(0);
        ballPusherMotor.setVelocity(0);
    }


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
        if (!opMode.opModeIsActive()) return;

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

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            // Display telemetry
            //telemetry.addData("Path", "Running to L:%7d R:%7d", leftFrontTarget, rightFrontTarget);
            //telemetry.addData("Current", "Running at L:%7d R:%7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            //telemetry.update();
            opMode.sleep(MINIMAL_SLEEP_TIME);
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
    public void strafe_right(double speed, double inches, double timeoutS) {
        // For strafing, the motor directions are:
        //Right: -LF, +LB, +RF, -RB
        // Left:  +LF, -LB, -RF, +RB
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);
    }

    public void strafe_left(double speed, double inches, double timeoutS) {
        // For strafing, the motor directions are:
        //Right: -LF, +LB, +RF, -RB
        // Left:  +LF, -LB, -RF, +RB
        encoderDrive(speed, -inches, inches, inches, -inches, timeoutS);
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
        if (!opMode.opModeIsActive()) return;



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

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()))
        {
            opMode.sleep(MINIMAL_SLEEP_TIME);
            // telemetry.addData("Path", "Strafing...");
            // telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
