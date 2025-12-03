package org.firstinspires.ftc.teamcode.logic;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.logic.CommonCamera_18954;
import org.firstinspires.ftc.teamcode.logic.CommonDefs;
import org.firstinspires.ftc.teamcode.logic.LimeLightHandler;
import org.firstinspires.ftc.teamcode.logic.LocalizerDecode;
import org.firstinspires.ftc.teamcode.logic.DriverIndicationLED;


public class Common_Teleop {


    // ---------------- HARDWARE DECLARATION ----------------
    DcMotor leftFront, rightFront, leftBack, rightBack;
	DcMotor launcherMotor;
    DcMotorEx ballPusherMotor, intakeMotor;
    Servo stopperServo;

    private CommonDefs.Alliance mAlliance;



    // ---------------- DRIVE SETTINGS ----------------
    private final double LOW_SPEED = 0.5;
    private final double NORMAL_SPEED = 0.75;
    private final double HIGH_SPEED = 1.0;
    private double speedMultiplier = NORMAL_SPEED;

    // ---------------- LAUNCHER SETTINGS ----------------
    public static long  LAUNCHER_SHORTTANGE_RPM = 2800;
    public static  long LAUNCHER_LONGRANGE_RPM = 5000;
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
    public static  final double GATE_UP_RAMP_FREE_SERVOPOS = 0.33;
    public static final double GATE_UP_RAMP_FREE_SERVOPOS_AUTON=GATE_UP_RAMP_FREE_SERVOPOS;


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
    public static  final long GATE_OPEN_MIN_TIME=800;
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

    CommonCamera_18954 mCameraRef;

    private final boolean ENABLE_CAMERA_DEFINE=false;
    private final boolean ENABLE_LIMEIGHT_CAMERA=true;


    private long Target_RPM_Shooting=0;

    LimeLightHandler mLimeLightHandler;
    IMU imu;

    private DriverIndicationLED mDriverIndicationLED;

    private LocalizerDecode mLocalizer;


    //Reference from OpMode
    OpMode opMode;
    hardwareMap hardwareMap;
    Telemetry telemetry;

    private void init_private()
    {
        // Private init tasks can be added here if needed
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

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));




        if(ENABLE_CAMERA_DEFINE) {
            mCameraRef = new CommonCamera_18954(this);
        }

        if(ENABLE_LIMEIGHT_CAMERA) {
            mLimeLightHandler = new LimeLightHandler(imu, hardwareMap,mAlliance);
        }

        mDriverIndicationLED=new DriverIndicationLED(hardwareMap);

        if(ENABLE_LIMEIGHT_CAMERA)
        {
            mLocalizer = new LocalizerDecode(mAlliance,mLimeLightHandler,mDriverIndicationLED);
        }
        else
        {
            mLocalizer = null;
        }


        telemetry.addData("Status", "Initialized");
    }

    // ---------------- INIT METHOD ----------------
    public void init(OpMode opMode,hardwareMap hardwareMap, Telemetry telemetry, CommonDefs.Alliance Alliance) {

        this.opMode=opMode;
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry;
        this.mAlliance = Alliance;
        init_private();        
    }

    // ---------------- LOOP METHOD ----------------

    public void loop() {

        Pose3D pose=null;
        LLResult limelight_result;
        double diff_percent=0;

        //----------------- CAMERA Feedback -----------------
        if(ENABLE_CAMERA_DEFINE) {
            pose = mCameraRef.telemetryAprilTag();
        }

        if(ENABLE_LIMEIGHT_CAMERA) {
            //pose = mLimeLightHandler.update(System.currentTimeMillis());
            limelight_result=mLocalizer.update(System.currentTimeMillis());
        }


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
                    currGatePos =  GatePosition.GATE_DOWN_PUSHED_BALL_IN;
                    shortRangeMode = shortPowerShot;

                }
                else
                {
                    currGatePos =  GatePosition.GATE_DOWN_PUSHED_BALL_IN;
                }
                break;



            case STARTING:
                if (System.currentTimeMillis() - stateStartTime >= INITIAL_SPIN_UP_TIME) { // spin-up time
                    shooterState = ShooterState.SHOOTER_GATE_UP_RAMP_FREE;
                    stateStartTime = System.currentTimeMillis();
                    currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
                }
                break;

                //Gate open
            case SHOOTER_GATE_UP_RAMP_FREE:

                if (gamepad2.dpad_down) {
                    shooterState = ShooterState.IDLE;
                    launcherOn = false;
                    currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
                    shortRangeMode = false;
                }
                else {
                        currGatePos = GatePosition.GATE_UP_RAMP_FREE;

                        if (System.currentTimeMillis() - stateStartTime >= GATE_OPEN_MIN_TIME) {
                            stateStartTime = System.currentTimeMillis();
                            shooterState = ShooterState.WAITING_FOR_RPMLOCK;
                        }
                }
                break;

            case WAITING_FOR_RPMLOCK:
            {
                if (shortRangeMode) {
                    Target_RPM_Shooting = LAUNCHER_SHORTTANGE_RPM;
                } else {
                    Target_RPM_Shooting = LAUNCHER_LONGRANGE_RPM;
                }

                if (gamepad2.dpad_down) {
                    shooterState = ShooterState.IDLE;
                    launcherOn = false;
                    currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
                    shortRangeMode = false;
                }
                else {

                    if ((Math.abs(getLauncherRpm() - Target_RPM_Shooting) <= (LAUNCHER_RPM_TOLERANCE))  && ((System.currentTimeMillis()-stateStartTime) >=600))
                    {
                        //push the ball in
                        currGatePos = GatePosition.GATE_DOWN_PUSHED_BALL_IN;
                        stateStartTime = System.currentTimeMillis();
                        shooterState = ShooterState.SHOOTING;
                    }
                    else if (System.currentTimeMillis() - stateStartTime >= MAX_WAITTIME_ACHIEVING_RPM) {
                        currGatePos = GatePosition.GATE_DOWN_PUSHED_BALL_IN;
                        stateStartTime = System.currentTimeMillis();
                        shooterState = ShooterState.TIMEOUT_SHOOTING;
                    }
                    else if (ForceShoot_WithoutRPM) {
                        //push the ball in
                        currGatePos = GatePosition.GATE_DOWN_PUSHED_BALL_IN;
                        stateStartTime = System.currentTimeMillis();
                        shooterState = ShooterState.FORCED_SHOOT;
                    }
                }
            }
            break;

            case TIMEOUT_SHOOTING:
            case FORCED_SHOOT:
            case SHOOTING:
            {
                    if (System.currentTimeMillis() - stateStartTime >= SHOOTING_POSITION_TIME) { // 0.5 sec close
                        shooterState = ShooterState.SHOOTER_GATE_UP_RAMP_FREE;
                        stateStartTime = System.currentTimeMillis();
                        currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
                    }

            }
            break;
        }

        if(gamepad2.x) {
            ForceShoot_WithoutRPM=false;
        }
        else if(gamepad2.b)
        {
            ForceShoot_WithoutRPM=true;
        }


        // ---------------- INTAKE ----------------
        intake_spitout=gamepad1.left_bumper;
        intakeOn = gamepad1.right_bumper;
        if(intakeOn) {
            intakeMotor.setPower(1.0);
        }
        else if(intake_spitout) {
            intakeMotor.setPower(-0.4);
        }
        else {
            intakeMotor.setPower(0.0);
        }

        // ---------------- BALL PUSHER ----------------
        ballPusherOn = launcherOn || intakeOn ;
        if(ballPusherOn) {
            ballPusherMotor.setPower(1.0);
        }
        else if(intake_spitout) {
            ballPusherMotor.setPower(-0.4);
        }
        else {
            ballPusherMotor.setPower(0.0);
        }


        // ----- RPM ADJUSTMENTS ----------

        //---------------RPM ADJUSTMENTS --- CONTROL
        if(gamepad1.x) {
            RPM_ADJUSTMENTS_ALLOWED = true;
        }
        else if(gamepad1.b) {
            RPM_ADJUSTMENTS_ALLOWED = false;
        }

        if(RPM_ADJUSTMENTS_ALLOWED) {
            if (SHORT_LAUNCHER_ADJUST_ACTIVE) {
                if (System.currentTimeMillis() - SHORT_LAUNCHER_LAST_ADJUST_TIME > LAUNCHER_ADJUST_HYSTERISIS) {
                    SHORT_LAUNCHER_ADJUST_ACTIVE = false;
                }
            }
            if (!SHORT_LAUNCHER_ADJUST_ACTIVE) {
                if (gamepad2.right_bumper) {
                    LAUNCHER_SHORTTANGE_RPM += 50;
                    SHORT_LAUNCHER_ADJUST_ACTIVE = true;
                    SHORT_LAUNCHER_LAST_ADJUST_TIME = System.currentTimeMillis();
                } else if (gamepad2.left_bumper) {
                    LAUNCHER_SHORTTANGE_RPM -= 50;
                    SHORT_LAUNCHER_ADJUST_ACTIVE = true;
                    SHORT_LAUNCHER_LAST_ADJUST_TIME = System.currentTimeMillis();
                }
            }

            if (LONG_LAUNCHER_ADJUST_ACTIVE) {
                if (System.currentTimeMillis() - LONG_LAUNCHER_LAST_ADJUST_TIME > LAUNCHER_ADJUST_HYSTERISIS) {
                    LONG_LAUNCHER_ADJUST_ACTIVE = false;
                }
            }
            if (!LONG_LAUNCHER_ADJUST_ACTIVE) {
                if (gamepad2.right_trigger > 0.5) {
                    LAUNCHER_LONGRANGE_RPM += 50;
                    LONG_LAUNCHER_ADJUST_ACTIVE = true;
                    LONG_LAUNCHER_LAST_ADJUST_TIME = System.currentTimeMillis();
                } else if (gamepad2.left_trigger > 0.5) {
                    LAUNCHER_LONGRANGE_RPM -= 50;
                    LONG_LAUNCHER_ADJUST_ACTIVE = true;
                    LONG_LAUNCHER_LAST_ADJUST_TIME = System.currentTimeMillis();
                }
            }
        }






        // ---------------- LAUNCHER CONTROL ----------------

        if(intake_spitout)
        {
            setLauncherRPM(-LAUNCHER_LONGRANGE_RPM);
            currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
        }
        else if (launcherOn) {

            double current_launcher_rpm;

            if (shortRangeMode) {
                Target_RPM_Shooting = LAUNCHER_SHORTTANGE_RPM;
            } else {
                Target_RPM_Shooting = LAUNCHER_LONGRANGE_RPM;
            }

            setLauncherRPM(Target_RPM_Shooting);

            current_launcher_rpm = getLauncherRpm();
            diff_percent = Math.abs(current_launcher_rpm - Target_RPM_Shooting) / Math.abs(Target_RPM_Shooting);

                
        } else {
            setLauncherRPM(0);
        }

        if(GATE_POSITION_TESTING_ENABLED == true)
        {
            if(System.currentTimeMillis() > GATE_POSITION_LASTADJUSTED_TIME + 500) {
                if (gamepad1.a) {
                    GATE_POSITION_TESTING += .1;
                    GATE_POSITION_LASTADJUSTED_TIME = System.currentTimeMillis();
                } else if (gamepad1.y) {
                    GATE_POSITION_TESTING -= .1;
                    GATE_POSITION_LASTADJUSTED_TIME = System.currentTimeMillis();
                }
            }
            stopperServo.setPosition(GATE_POSITION_TESTING);
        }
        else {

            if(currGatePos == GatePosition.GATE_DOWN_PUSHED_BALL_IN) {
                stopperServo.setPosition(GATE_DOWN_PUSHED_BALL_IN_SERVOPOS);
            }
            else {
                stopperServo.setPosition(GATE_UP_RAMP_FREE_SERVOPOS);
            }
        }


        // ---------------- TELEMETRY ----------------
        //telemetry.addData("Speed Multiplier", String.format("%.2f", speedMultiplier));
        //telemetry.addData("GateClosed", currGatePos.toString());
        //telemetry.addData("Ball Pusher", ballPusherOn ? "Running" : "Stopped");
        //telemetry.addData("Launcher", launcherOn ? "Running" : "Stopped");
        //telemetry.addData("Launcher Mode", shortRangeMode ? "SHORT RANGE (15%)" : "FULL POWER");
        //telemetry.addData("Intake", intakeOn ? "Running" : "Stopped");
        //telemetry.addData("Shooter State", shooterState.toString());
        telemetry.addData("Launcher RPM",getLauncherRpm());
        telemetry.addData("Differential",diff_percent);
        telemetry.addData("RPM Based Shooting",!ForceShoot_WithoutRPM);
        telemetry.addData("Short Range RPM",LAUNCHER_SHORTTANGE_RPM);
        telemetry.addData("Long Range RPM",LAUNCHER_LONGRANGE_RPM);
        telemetry.addData("RPM Modifiable ?",RPM_ADJUSTMENTS_ALLOWED);
        //telemetry.addData("GATE_POSITION_TESTING_ENABLED",GATE_POSITION_TESTING_ENABLED);
        //telemetry.addData("GATE_POSITION_TESTING ?",GATE_POSITION_TESTING);

        if(ENABLE_LIMEIGHT_CAMERA){
            if(limelight_result != null) {
                pose=mLimeLightHandler.getLast_botpose();
                telemetry.addData("tx", String.format("%.3f", (limelight_result.getTx())));
                telemetry.addData("X", CommonDefs.ConvertCameraPosToInches_x(pose.getPosition().x));
                telemetry.addData("Y", CommonDefs.ConvertCameraPosToInches_y(pose.getPosition().y));
                telemetry.addData("Z", CommonDefs.ConvertCameraPosToInches_z(pose.getPosition().z));
                telemetry.addData("Heading", pose.getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch", pose.getOrientation().getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll", pose.getOrientation().getRoll(AngleUnit.DEGREES));
            }
        }


        if(ENABLE_CAMERA_DEFINE) {

            if (pose != null) {
                telemetry.addData("X", pose.getPosition().x);
                telemetry.addData("Y", pose.getPosition().y);
                telemetry.addData("Z", pose.getPosition().z);
                telemetry.addData("Heading", pose.getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch", pose.getOrientation().getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll", pose.getOrientation().getRoll(AngleUnit.DEGREES));
            }
        }

        telemetry.update();
    }


    public long getLauncherRpm() {
        // First, check if launcherMotor is a DcMotorEx instance to safely call getVelocity()
        if (launcherMotor instanceof DcMotorEx) {
            // getVelocity() returns the speed in "ticks per second"
            double ticksPerSecond = ((DcMotorEx) launcherMotor).getVelocity();

            // Convert ticks per second to revolutions per minute
            // (ticks/sec) * (60 sec/min) / (ticks/rev) = rev/min
            return (long)((ticksPerSecond * 60) / LAUNCHER_MOTOR_TICKS_PER_REV);
        }
        // Return 0 if it's not a DcMotorEx or if something is wrong
        return 0;
    }

    public boolean setLauncherRPM(long RPMToSet) {
        // First, check if launcherMotor is a DcMotorEx instance to safely call getVelocity()
        if (launcherMotor instanceof DcMotorEx) {
            double AngularVelocity;

            AngularVelocity=(RPMToSet * LAUNCHER_MOTOR_TICKS_PER_REV)/60 ;

             ((DcMotorEx) launcherMotor).setVelocity(AngularVelocity);
            return  true;
        }
        // Return 0 if it's not a DcMotorEx or if something is wrong
        return false;

    }
    
}
