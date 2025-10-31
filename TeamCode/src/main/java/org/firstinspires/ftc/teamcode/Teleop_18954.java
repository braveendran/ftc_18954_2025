package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Teleop", group = "Test")
public class Teleop_18954 extends OpMode {

    // ---------------- HARDWARE DECLARATION ----------------
    DcMotor leftFront, rightFront, leftBack, rightBack;
	DcMotor launcherMotor;
    DcMotorEx ballPusherMotor, intakeMotor;
    Servo stopperServo;

    private boolean isBlueTeam=true;

    private CommonFunc_18954 objCommonFunc;

    private long INITIAL_SPIN_UP_TIME=1300;
    private long GATE_DOWN_TIME=500;
    private long GATE_UP_TIME=800;

    // ---------------- DRIVE SETTINGS ----------------
    private final double LOW_SPEED = 0.5;
    private final double NORMAL_SPEED = 0.75;
    private final double HIGH_SPEED = 1.0;
    private double speedMultiplier = NORMAL_SPEED;

    // ---------------- LAUNCHER SETTINGS ----------------
    private final double LAUNCHER_MAX_VELOCITY = 1.0;

    private final double LAUNCHER_LONG_RANGE_VELOCITY = LAUNCHER_MAX_VELOCITY*.98;

    private double LAUNCHER_SHORT_RANGE_VELOCITY = LAUNCHER_MAX_VELOCITY * 0.80;

    // ---------------- STOPPER SETTINGS ----------------
    private final double STOPPER_CLOSED = 0.70;
    private final double STOPPER_OPEN = .94;

    // ---------------- CONTROL FLAGS ----------------
    private boolean launcherOn = false;
    private boolean intakeOn = false;
    private boolean ballPusherOn = false;
    private boolean stopperOpen = false;

    private enum ShooterState {
        IDLE, STARTING, FEED_OPEN, FEED_CLOSE
    }
    private ShooterState shooterState = ShooterState.IDLE;
    private long stateStartTime = 0;
    private boolean shortRangeMode = false;

    CommonCamera_18954 mCameraRef;

    // ---------------- INIT METHOD ----------------
    @Override
    public void init() {
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
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        ballPusherMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ballPusherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Launcher encoder mode
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stopper initial position
        stopperServo.setPosition(STOPPER_CLOSED);

        mCameraRef=new CommonCamera_18954(this);


        telemetry.addData("Status", "Initialized");
    }

    // ---------------- LOOP METHOD ----------------
    @Override
    public void loop() {

        //----------------- CAMERA Feedback -----------------
        Pose3D pose=mCameraRef.telemetryAprilTag(isBlueTeam);


        // ---------------- DRIVE CONTROL ----------------
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.1) {
            speedMultiplier = HIGH_SPEED;
        } else if (gamepad1.left_trigger > 0.1) {
            speedMultiplier = LOW_SPEED;
        } else {
            speedMultiplier = NORMAL_SPEED;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower((y + x + rx) / denominator * speedMultiplier);
        leftBack.setPower((y - x + rx) / denominator * speedMultiplier);
        rightFront.setPower((y - x - rx) / denominator * speedMultiplier);
        rightBack.setPower((y + x - rx) / denominator * speedMultiplier);

        // ---------------- SHOOTER SEQUENCE ----------------
        boolean fullPowerShot = gamepad2.a;
        boolean shortPowerShot = gamepad2.y;

        switch (shooterState) {
            case IDLE:
                if (fullPowerShot || shortPowerShot) {
                    shooterState = ShooterState.STARTING;
                    stateStartTime = System.currentTimeMillis();
                    launcherOn = true;
                    stopperOpen = false;
                    shortRangeMode = shortPowerShot;
                }
                break;



            case STARTING:
                if (System.currentTimeMillis() - stateStartTime >= INITIAL_SPIN_UP_TIME) { // spin-up time
                    shooterState = ShooterState.FEED_OPEN;
                    stateStartTime = System.currentTimeMillis();
                    stopperOpen = true;
                }
                break;

            case FEED_OPEN:
                if (System.currentTimeMillis() - stateStartTime >= GATE_DOWN_TIME) { // 0.5 sec open
                    shooterState = ShooterState.FEED_CLOSE;
                    stateStartTime = System.currentTimeMillis();
                    stopperOpen = false;
                }
                break;

            case FEED_CLOSE:
                if (System.currentTimeMillis() - stateStartTime >= GATE_UP_TIME) { // 0.5 sec close
                    if (!gamepad2.dpad_down) {
                        shooterState = ShooterState.FEED_OPEN;
                        stateStartTime = System.currentTimeMillis();
                        stopperOpen = true;
                    } else {
                        shooterState = ShooterState.IDLE;
                        launcherOn = false;
                        stopperOpen = false;
                        shortRangeMode = false;
                    }
                }
                break;
        }

        // ---------------- BALL PUSHER ----------------
        ballPusherOn = (shooterState != ShooterState.IDLE) || (gamepad1.right_trigger > 0.1);
        ballPusherMotor.setPower(ballPusherOn ? 1.0 : 0.0);

        // ---------------- INTAKE ----------------
        intakeOn = (gamepad1.right_trigger > 0.1);
        intakeMotor.setPower(intakeOn ? 1.0 : 0.0);

        // ---------------- LAUNCHER CONTROL ----------------
        if (shooterState == ShooterState.IDLE) {
            launcherOn = gamepad2.right_trigger > 1;
            shortRangeMode = false;
        }

        if (launcherOn) {
            double powertoset = shortRangeMode ? LAUNCHER_SHORT_RANGE_VELOCITY : LAUNCHER_LONG_RANGE_VELOCITY;
			launcherMotor.setPower(powertoset);
                
        } else {
                launcherMotor.setPower(0);
        }

        // ---------------- STOPPER CONTROL ----------------
        if (shooterState == ShooterState.IDLE) {
            if (gamepad2.right_bumper) {
                stopperOpen = true;
            } else if (gamepad2.left_bumper) {
                stopperOpen = false;
            }
        }

        stopperServo.setPosition(stopperOpen ? STOPPER_OPEN : STOPPER_CLOSED);

        // ---------------- TELEMETRY ----------------
        telemetry.addData("Speed Multiplier", String.format("%.2f", speedMultiplier));
        telemetry.addData("Stopper", stopperOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Ball Pusher", ballPusherOn ? "Running" : "Stopped");
        telemetry.addData("Launcher", launcherOn ? "Running" : "Stopped");
        telemetry.addData("Launcher Mode", shortRangeMode ? "SHORT RANGE (15%)" : "FULL POWER");
        telemetry.addData("Long Range Power ", LAUNCHER_LONG_RANGE_VELOCITY);
        telemetry.addData("Short Range Power", LAUNCHER_SHORT_RANGE_VELOCITY);
        telemetry.addData("Intake", intakeOn ? "Running" : "Stopped");
        telemetry.addData("Shooter State", shooterState.toString());

        if(pose!=null) {
            telemetry.addData("X", pose.getPosition().x);
            telemetry.addData("Y",  pose.getPosition().y);
            telemetry.addData("Z",  pose.getPosition().z);
            telemetry.addData("Heading", pose.getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", pose.getOrientation().getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", pose.getOrientation().getRoll(AngleUnit.DEGREES));
        }

        telemetry.update();
    }
}