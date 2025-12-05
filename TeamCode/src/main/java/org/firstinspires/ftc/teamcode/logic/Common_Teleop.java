package org.firstinspires.ftc.teamcode.logic;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.logic.CommonDefs;
import org.firstinspires.ftc.teamcode.logic.LimeLightHandler;
import org.firstinspires.ftc.teamcode.logic.LocalizerDecode;
import org.firstinspires.ftc.teamcode.logic.DriverIndicationLED;
import org.firstinspires.ftc.teamcode.logic.DistVelocityProjection;


public class Common_Teleop {


    // ---------------- HARDWARE DECLARATION ----------------
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor launcherMotor;
    DcMotor lateralEncoder; // Forward odometry pod
    DcMotorEx ballPusherMotor, intakeMotor; // ballPusherMotor also serves as strafer pod
    Servo stopperServo;

    private CommonDefs.Alliance mAlliance;



    // ---------------- DRIVE SETTINGS ----------------
    private final double LOW_SPEED = 0.5;
    private final double NORMAL_SPEED = 0.75;
    private final double HIGH_SPEED = 1.0;
    private double speedMultiplier = NORMAL_SPEED;

    // ---------------- LAUNCHER SETTINGS ----------------
    public static long  LAUNCHER_SHORTTANGE_RPM = 2800;
    public static  long LAUNCHER_LONGRANGE_RPM = 3500;
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
        IDLE, STARTING,STARTING_TURNING_TIMEDOUT,  SHOOTER_GATE_UP_RAMP_FREE, WAITING_FOR_RPMLOCK, SHOOTING, TIMEOUT_SHOOTING , FORCED_SHOOT ,TURNING_TO_SHOOT
    }

    private GatePosition currGatePos = GatePosition.GATE_UP_RAMP_FREE;
    private ShooterState shooterState = ShooterState.IDLE;

    public static  final  long INITIAL_SPIN_UP_TIME=900;
    public static final long  SHOOTING_TURN_TIME_THRESHOLD=1300;
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
    private boolean CancelShootingAfterCurrentSequence=false;


    private final boolean ENABLE_LIMEIGHT_CAMERA=true;


    private long Target_RPM_Shooting=0;

    LimeLightHandler mLimeLightHandler;
    IMU imu;

    private DriverIndicationLED mDriverIndicationLED;

    private LocalizerDecode mLocalizer;
    private DistVelocityProjection mDistVelocityProjection;
    
    // System timer
    private long systemStartTime = 0;
    private boolean systemTimerStarted = false;

    //Reference from OpMode
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private double turn_relative_targetYaw;
    private double turn_relative_currentYaw;

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
        lateralEncoder = hardwareMap.dcMotor.get("lateralencoder");
        //launcherBottomMotor = hardwareMap.dcMotor.get("LauncherBottomMotor");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        stopperServo = hardwareMap.get(Servo.class, "stopperServo");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

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



        if(ENABLE_LIMEIGHT_CAMERA) {
            mLimeLightHandler = new LimeLightHandler(imu, hardwareMap,mAlliance);
        }

        mDriverIndicationLED=new DriverIndicationLED(hardwareMap);

        if(ENABLE_LIMEIGHT_CAMERA)
        {
            mLocalizer = new LocalizerDecode(mAlliance, mLimeLightHandler, mDriverIndicationLED, 
                                           lateralEncoder, ballPusherMotor);
        }
        else
        {
            mLocalizer = null;
        }
        
        // Initialize distance-velocity projection for dynamic RPM
        mDistVelocityProjection = new DistVelocityProjection();

        CancelShootingAfterCurrentSequence=false;

        telemetry.addData("Status", "Initialized");
    }

    // ---------------- INIT METHOD ----------------
    public void init(OpMode opMode,CommonDefs.Alliance Alliance) {

        this.opMode=opMode;
        this.hardwareMap=opMode.hardwareMap;
        this.telemetry=opMode.telemetry;
        this.mAlliance = Alliance;
        init_private();        
    }


    private void DriveControl_PowerBased(double y, double x, double rx)
    {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower((y + x + rx) / denominator * speedMultiplier);
        leftBack.setPower((y - x + rx) / denominator * speedMultiplier);
        rightFront.setPower((y - x - rx) / denominator * speedMultiplier);
        rightBack.setPower((y + x - rx) / denominator * speedMultiplier);
    }

    // ---------------- LOOP METHOD ----------------

    public void loop() {
        
        // Start system timer on first loop iteration
        if (!systemTimerStarted) {
            systemStartTime = System.currentTimeMillis();
            systemTimerStarted = true;
        }

        Pose3D pose=null;
        LLResult limelight_result = null;
        double diff_percent=0;

        //----------------- CAMERA Feedback -----------------

        if(ENABLE_LIMEIGHT_CAMERA) {
            //pose = mLimeLightHandler.update(System.currentTimeMillis());
            limelight_result=mLocalizer.update(System.currentTimeMillis());
        }


        // ---------------- DRIVE CONTROL ----------------
        double y = -this.opMode.gamepad1.left_stick_y;
        double x = this.opMode.gamepad1.left_stick_x * 1.1;
        double rx = this.opMode.gamepad1.right_stick_x;

        if (this.opMode.gamepad1.right_trigger > 0.1) {
            speedMultiplier = HIGH_SPEED;
        } else if (this.opMode.gamepad1.left_trigger > 0.1) {
            speedMultiplier = LOW_SPEED;
        } else {
            speedMultiplier = NORMAL_SPEED;
        }


        if(shooterState == ShooterState.TURNING_TO_SHOOT)
        {
            //turning will be controlled in the state machine
        }
        else
        {
            //Drive control using the gamepad
            DriveControl_PowerBased(y, x, rx);
        }
        



        // ---------------- SHOOTER SEQUENCE ----------------
        boolean fullPowerShot = this.opMode.gamepad2.a || this.opMode.gamepad2.left_bumper || (this.opMode.gamepad2.left_trigger>0.5);
        boolean shortPowerShot = this.opMode.gamepad2.y || this.opMode.gamepad2.right_bumper || (this.opMode.gamepad2.left_trigger>0.5);
        boolean turn_before_shoot=false;
        boolean dynamicRPM_distancebased=false;
        double turn_angle_shoot_correction=0;
        if(
            (
                ( this.opMode.gamepad2.left_bumper || this.opMode.gamepad2.right_bumper )  ||
                ((this.opMode.gamepad2.left_trigger>0.5) ||  (this.opMode.gamepad2.left_trigger>0.5) ) 
            )
            &&  (limelight_result != null && limelight_result.isValid()) 
        )
        {
            turn_before_shoot=true;
            turn_angle_shoot_correction=mLocalizer.getHeadingCorrectionDeg();            
        }

        if( ((this.opMode.gamepad2.left_trigger>0.5) ||  (this.opMode.gamepad2.left_trigger>0.5) ) )
        {
            dynamicRPM_distancebased=true;
        }        

        if (this.opMode.gamepad2.dpad_down)
        {
            CancelShootingAfterCurrentSequence=true;
        }

        switch (shooterState) {
            case IDLE:
                if (fullPowerShot || shortPowerShot) {
                    if(turn_before_shoot)
                    {
                        turn_relative_start(turn_angle_shoot_correction);
                        shooterState = ShooterState.TURNING_TO_SHOOT;
                    }
                    else
                    {
                        shooterState = ShooterState.STARTING;
                    }
                    stateStartTime = System.currentTimeMillis();
                    launcherOn = true;
                    currGatePos =  GatePosition.GATE_DOWN_PUSHED_BALL_IN;
                    shortRangeMode = shortPowerShot;
                    CancelShootingAfterCurrentSequence=false;

                }
                else
                {
                    currGatePos =  GatePosition.GATE_DOWN_PUSHED_BALL_IN;
                }
                break;

            case TURNING_TO_SHOOT:
                if (System.currentTimeMillis() - stateStartTime >= SHOOTING_TURN_TIME_THRESHOLD)
                {
                    turn_relative_stop();
                    shooterState = ShooterState.STARTING_TURNING_TIMEDOUT;
                    stateStartTime = System.currentTimeMillis();
                    
                }
                else{
                    if(turn_relative_main())
                    {
                        turn_relative_stop();
                        shooterState = ShooterState.STARTING;
                        stateStartTime = System.currentTimeMillis();
                    }                
                }
                break;


            case STARTING_TURNING_TIMEDOUT:
            case STARTING:
                if (System.currentTimeMillis() - stateStartTime >= INITIAL_SPIN_UP_TIME) { // spin-up time
                    shooterState = ShooterState.SHOOTER_GATE_UP_RAMP_FREE;
                    stateStartTime = System.currentTimeMillis();
                    currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
                }
                break;

                //Gate open
            case SHOOTER_GATE_UP_RAMP_FREE: 
                
                currGatePos = GatePosition.GATE_UP_RAMP_FREE;
                if (System.currentTimeMillis() - stateStartTime >= GATE_OPEN_MIN_TIME) {
                    stateStartTime = System.currentTimeMillis();
                    shooterState = ShooterState.WAITING_FOR_RPMLOCK;
                }
                
                break;

            case WAITING_FOR_RPMLOCK:
            {
                if (dynamicRPM_distancebased && mLocalizer != null) {
                    // Use dynamic RPM based on distance to target
                    Pose fusedPos = mLocalizer.getCurrentFusedPosition();
                    double distanceToTarget = (mAlliance == CommonDefs.Alliance.RED) ?
                        mLocalizer.getPositionLocalizer().getDistanceToRedBasket() :
                        mLocalizer.getPositionLocalizer().getDistanceToBlueBasket();
                    Target_RPM_Shooting = (long)mDistVelocityProjection.getVelocity(distanceToTarget);
                } else if (shortRangeMode) {
                    Target_RPM_Shooting = LAUNCHER_SHORTTANGE_RPM;
                } else {
                    Target_RPM_Shooting = LAUNCHER_LONGRANGE_RPM;
                }

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
            break;

            case TIMEOUT_SHOOTING:
            case FORCED_SHOOT:
            case SHOOTING:
            {               
                    if (System.currentTimeMillis() - stateStartTime >= SHOOTING_POSITION_TIME) { // 0.5 sec close
                        if (CancelShootingAfterCurrentSequence)
                        {
                            CancelShootingAfterCurrentSequence=false;
                            shooterState = ShooterState.IDLE;
                            launcherOn = false;
                            currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
                            shortRangeMode = false;
                        }
                        else
                        {
                            shooterState = ShooterState.SHOOTER_GATE_UP_RAMP_FREE;
                            stateStartTime = System.currentTimeMillis();
                            currGatePos =  GatePosition.GATE_UP_RAMP_FREE;
                        }
                    }

            }
            break;
        }

        if(this.opMode.gamepad2.x) {
            ForceShoot_WithoutRPM=false;
        }
        else if(this.opMode.gamepad2.b)
        {
            ForceShoot_WithoutRPM=true;
        }


        // ---------------- INTAKE ----------------
        intake_spitout=this.opMode.gamepad1.left_bumper;
        intakeOn = this.opMode.gamepad1.right_bumper;
        if(intakeOn) {
            intakeMotor.setPower(1.0);
        }
        else if(intake_spitout) {
            intakeMotor.setPower(-1.0);
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
            ballPusherMotor.setPower(-1.0);
        }
        else {
            ballPusherMotor.setPower(0.0);
        }


        // ----- RPM ADJUSTMENTS ----------

        //---------------RPM ADJUSTMENTS --- CONTROL
        if(this.opMode.gamepad1.x) {
            RPM_ADJUSTMENTS_ALLOWED = true;
        }
        else if(this.opMode.gamepad1.b) {
            RPM_ADJUSTMENTS_ALLOWED = false;
        }

        if(RPM_ADJUSTMENTS_ALLOWED) {
            if (SHORT_LAUNCHER_ADJUST_ACTIVE) {
                if (System.currentTimeMillis() - SHORT_LAUNCHER_LAST_ADJUST_TIME > LAUNCHER_ADJUST_HYSTERISIS) {
                    SHORT_LAUNCHER_ADJUST_ACTIVE = false;
                }
            }
            if (!SHORT_LAUNCHER_ADJUST_ACTIVE) {
                if (this.opMode.gamepad2.left_stick_y < -0.5) {
                    LAUNCHER_SHORTTANGE_RPM += 50;
                    SHORT_LAUNCHER_ADJUST_ACTIVE = true;
                    SHORT_LAUNCHER_LAST_ADJUST_TIME = System.currentTimeMillis();
                } else if (this.opMode.gamepad2.left_stick_y > 0.5) {
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
                if (this.opMode.gamepad2.right_stick_y < -0.5) {
                    LAUNCHER_LONGRANGE_RPM += 50;
                    LONG_LAUNCHER_ADJUST_ACTIVE = true;
                    LONG_LAUNCHER_LAST_ADJUST_TIME = System.currentTimeMillis();
                } else if (this.opMode.gamepad2.right_stick_y > 0.5 ) {
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

            if (dynamicRPM_distancebased && mLocalizer != null) {
                // Use dynamic RPM based on distance to target
                Pose fusedPos = mLocalizer.getCurrentFusedPosition();
                double distanceToTarget = (mAlliance == CommonDefs.Alliance.RED) ?
                    mLocalizer.getPositionLocalizer().getDistanceToRedBasket() :
                    mLocalizer.getPositionLocalizer().getDistanceToBlueBasket();
                Target_RPM_Shooting = (long)mDistVelocityProjection.getVelocity(distanceToTarget);
            } else if (shortRangeMode) {
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
                if (this.opMode.gamepad1.a) {
                    GATE_POSITION_TESTING += .1;
                    GATE_POSITION_LASTADJUSTED_TIME = System.currentTimeMillis();
                } else if (this.opMode.gamepad1.y) {
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
        telemetry.addData("Shooter State", shooterState.toString());
        telemetry.addData("Launcher RPM",getLauncherRpm());
        telemetry.addData("Differential",diff_percent);
        telemetry.addData("RPM Based Shooting",!ForceShoot_WithoutRPM);
        telemetry.addData("Short Range RPM",LAUNCHER_SHORTTANGE_RPM);
        telemetry.addData("Long Range RPM",LAUNCHER_LONGRANGE_RPM);
        telemetry.addData("RPM Modifiable ?",RPM_ADJUSTMENTS_ALLOWED);
        
        // System timer
        long elapsedTime = System.currentTimeMillis() - systemStartTime;
        telemetry.addData("System Time", String.format("%.1fs", elapsedTime / 1000.0));
        
        // Position tracking telemetry
        if (mLocalizer != null) {
            telemetry.addData("Pos Fused", mLocalizer.getFusedPositionString());
            telemetry.addData("Pos Camera", mLocalizer.getCameraPositionString());
            telemetry.addData("Pos Encoder", mLocalizer.getEncoderPositionString());
        }
        //telemetry.addData("GATE_POSITION_TESTING_ENABLED",GATE_POSITION_TESTING_ENABLED);
        //telemetry.addData("GATE_POSITION_TESTING ?",GATE_POSITION_TESTING);
        if(imu != null)
        {
            telemetry.addData("IMU Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }

        if(ENABLE_LIMEIGHT_CAMERA){
            if(limelight_result != null && limelight_result.isValid())  {
                pose=mLimeLightHandler.getLast_botpose();
                telemetry.addData("tx", String.format("%.3f", (limelight_result.getTx())));
                telemetry.addData("heading corr", String.format("%.3f", mLocalizer.getHeadingCorrectionDeg()));
                telemetry.addData("X", CommonDefs.ConvertCameraPosToInches_x(pose.getPosition().x));
                telemetry.addData("Y", CommonDefs.ConvertCameraPosToInches_y(pose.getPosition().y));
 //               telemetry.addData("Z", CommonDefs.ConvertCameraPosToInches_z(pose.getPosition().z));
                telemetry.addData("Heading", pose.getOrientation().getYaw(AngleUnit.DEGREES));
 //               telemetry.addData("Pitch", pose.getOrientation().getPitch(AngleUnit.DEGREES));
 //               telemetry.addData("Roll", pose.getOrientation().getRoll(AngleUnit.DEGREES));
                telemetry.addData("turn_relative_currentYaw", turn_relative_currentYaw);
                telemetry.addData("turn_relative_targetYaw", turn_relative_targetYaw);
                
                if (dynamicRPM_distancebased && mLocalizer != null) {
                    double distanceToTarget = (mAlliance == CommonDefs.Alliance.RED) ?
                        mLocalizer.getPositionLocalizer().getDistanceToRedBasket() :
                        mLocalizer.getPositionLocalizer().getDistanceToBlueBasket();
                    telemetry.addData("Target Distance", String.format("%.1f in", distanceToTarget));
                    telemetry.addData("Dynamic RPM", (long)mDistVelocityProjection.getVelocity(distanceToTarget));
                }

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


    private void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    private void turn_relative_stop() {
        // Stop motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        //setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private boolean turn_relative_main()
    {

        boolean turn_completed=false;        

        if (imu == null) {
            //feature unavailable
            turn_completed=true;
            return turn_completed;
        }

        final double HEADING_TOLERANCE_DEG = CommonDefs.LIMELIGHT_HEADING_SHOOT_TOLERANCE_CLOSE; // degrees
        final double MIN_POWER = 0.10; // minimum power to overcome static friction
        final double K_P = 0.015; // proportional gain (tune for your robot)

        java.util.function.DoubleUnaryOperator norm = (v) -> {
            double a = v % 360.0;
            if (a <= -180.0) a += 360.0;
            if (a > 180.0) a -= 360.0;
            return a;
        };


        java.util.function.BiFunction<Double, Double, Double> shortestDiff = (target, current) -> {
            double diff = norm.applyAsDouble(target - current);
            return diff;
        };
    

        turn_relative_currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = shortestDiff.apply(turn_relative_targetYaw, turn_relative_currentYaw);

        // If within tolerance, we're done
        if (Math.abs(error) <=  CommonDefs.LIMELIGHT_HEADING_SHOOT_TOLERANCE_CLOSE) 
        {
            turn_completed=true;
            return turn_completed;
        }

        // P-controller output
        double output = K_P * error;
        double absOut = Math.abs(output);
        if (absOut < MIN_POWER) absOut = MIN_POWER;

        double leftPower = 0.0;
        double rightPower = 0.0;
        if (error > 0) {
            // Need to turn left: left wheels reverse, right wheels forward
            leftPower = -absOut;
            rightPower = absOut;
        } else {
                // Need to turn right
            leftPower = absOut;
            rightPower = -absOut;
        }

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        // Update Limelight (if present) to keep vision data fresh while turning
        if (mLimeLightHandler != null) {
            try {
                mLimeLightHandler.update(System.currentTimeMillis());
            } catch (Exception ignored) {}
        }
        return turn_completed;
    }

    private boolean turn_relative_start(double angle) {
        // Prefer IMU-based closed-loop turning, with Limelight updates while turning.
        // If IMU is not initialized, fall back to the encoder-based approximation.
        boolean turn_completed=false;        

        if (imu == null) {
            //feature unavailable
            turn_completed=true;
            return turn_completed;
        }

        //setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Helper lambdas for angle normalization and shortest difference
        java.util.function.DoubleUnaryOperator norm = (v) -> {
            double a = v % 360.0;
            if (a <= -180.0) a += 360.0;
            if (a > 180.0) a -= 360.0;
            return a;
        };

 

        // Read current heading (degrees)
        turn_relative_currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turn_relative_targetYaw = norm.applyAsDouble(turn_relative_currentYaw + angle);


        long startMs = System.currentTimeMillis();

        // Ensure motors are in a mode suitable for setting power directly
        turn_completed=turn_relative_main();
        return turn_completed;
    }
    
}
