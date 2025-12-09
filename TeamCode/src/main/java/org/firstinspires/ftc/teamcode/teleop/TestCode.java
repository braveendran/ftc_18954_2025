package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "TestCode", group = "Test")

public class TestCode extends OpMode {

    // ---------------- HARDWARE DECLARATION ----------------
    DcMotor leftFront, rightFront, leftBack, rightBack;
	DcMotor launcherMotor;
    DcMotorEx ballPusherMotor, intakeMotor;
    Servo stopperServo;

    private boolean isBlueTeam=true;



    // ---------------- DRIVE SETTINGS ----------------
    private final double LOW_SPEED = 0.5;
    private final double NORMAL_SPEED = 0.75;
    private final double HIGH_SPEED = 1.0;
    private double speedMultiplier = NORMAL_SPEED;

    // ---------------- LAUNCHER SETTINGS ----------------
    public static long  LAUNCHER_SHORTTANGE_RPM = 2850;
    public static  long LAUNCHER_LONGRANGE_RPM = 3600;
    public static final  long LAUNCHER_RPM_TOLERANCE = 100;




    private boolean SHORT_LAUNCHER_ADJUST_ACTIVE=false;
    private boolean LONG_LAUNCHER_ADJUST_ACTIVE=false;
    private long SHORT_LAUNCHER_LAST_ADJUST_TIME=0;
    private long LONG_LAUNCHER_LAST_ADJUST_TIME=0;

    private final double LAUNCHER_ADJUST_HYSTERISIS=500;
    private boolean RPM_ADJUSTMENTS_ALLOWED=false;

    private final double LAUNCHER_MOTOR_TICKS_PER_REV = 28.0;





    // ---------------- STOPPER SETTINGS ----------------
    public static final double GATE_DOWN_PUSHED_BALL_IN_SERVOPOS = 0.83;
    public static  final double GATE_UP_RAMP_FREE_SERVOPOS = 0.4;


    private enum GatePosition {
        GATE_DOWN_PUSHED_BALL_IN, GATE_UP_RAMP_FREE
    }

    private enum ShooterState {
        IDLE, STARTING,  SHOOTER_GATE_UP_RAMP_FREE, WAITING_FOR_RPMLOCK, SHOOTING, TIMEOUT_SHOOTING , FORCED_SHOOT
    }

    private GatePosition currGatePos = GatePosition.GATE_UP_RAMP_FREE;
    private ShooterState shooterState = ShooterState.IDLE;

    public static  final  long INITIAL_SPIN_UP_TIME=900;
    public static  final long SHOOTING_POSITION_TIME=600;
    public static  final long GATE_OPEN_MIN_TIME=500;
    public static  final long MAX_WAITTIME_ACHIEVING_RPM=500;








    private double GATE_POSITION_TESTING = 0.5;
    private boolean GATE_POSITION_TESTING_ENABLED=false;
    private long GATE_POSITION_LASTADJUSTED_TIME=0;

    // ---------------- CONTROL FLAGS ----------------
    private boolean launcherOn = false;

    public boolean intake_spitout=false;
    private boolean intakeOn = false;
    private boolean ballPusherOn = false;


    private boolean ForceShoot_WithoutRPM=false;



    private long stateStartTime = 0;
    private boolean shortRangeMode = false;


    private final boolean ENABLE_CAMERA_DEFINE=false;

    private long Target_RPM_Shooting=0;









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
        //launcherBottomMotor = hardwareMap.dcMotor.get("LauncherBottomMotor");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        stopperServo = hardwareMap.get(Servo.class, "stopperServo");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        ballPusherMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);
        //launcherBottomMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ballPusherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //launcherBottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Launcher encoder mode
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //launcherBottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stopper initial position
        stopperServo.setPosition(GATE_DOWN_PUSHED_BALL_IN_SERVOPOS);



        telemetry.addData("Status", "Initialized");
    }

    // ---------------- LOOP METHOD ----------------
    @Override
    public void loop() {

        Pose3D pose;
        double diff_percent = 0;

        if(gamepad1.y)
        {
            rightFront.setPower(1);
        }
        else {
            rightFront.setPower(0);
        }

        if(gamepad1.b)
        {
            rightBack.setPower(1);
        }
        else {
            rightBack.setPower(0);
        }

        if(gamepad1.x)
        {
            leftFront.setPower(1);
        }
        else {
            leftFront.setPower(0);
        }

        if(gamepad1.a)
        {
            leftBack.setPower(1);
        }
        else {
            leftBack.setPower(0);
        }

    }
}