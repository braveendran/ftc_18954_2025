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
	DcMotor launcherMotor, launcherBottomMotor;
    DcMotorEx ballPusherMotor, intakeMotor;
    Servo stopperServo;

    private boolean isBlueTeam=true;

    private CommonFunc_18954 objCommonFunc;

    private long INITIAL_SPIN_UP_TIME=2000;
    private long GATE_OPEN_TIME=500;
    private long GATE_CLOSED_TIME=800;

    // ---------------- DRIVE SETTINGS ----------------
    private final double LOW_SPEED = 0.5;
    private final double NORMAL_SPEED = 0.75;
    private final double HIGH_SPEED = 1.0;
    private double speedMultiplier = NORMAL_SPEED;

    // ---------------- LAUNCHER SETTINGS ----------------
    private final double LAUNCHER_MAX_POWER = 1.0;

    private double LAUNCHER_LONG_RANGE_POWER = LAUNCHER_MAX_POWER;
    private double LAUNCHER_SHORT_RANGE_POWER = LAUNCHER_MAX_POWER * 0.62;

    private final double  LAUNCHER_POWER_ADJUSTER = .03;
    private long LAUNCHER_SHORTTANGE_RPM = 3600;
    private long LAUNCHER_LONGRANGE_RPM = 4500;
    private final long LAUNCHER_RPM_TOLERANCE = 200;


    private final double LAUNCHER_BACK_RATIO=1.2;

    private final double LAUNCHER_SHORT_RANGE_MIN=0.2;
    private final double LAUNCHER_SHORT_RANGE_MAX=1.0;

    private boolean SHORT_LAUNCHER_ADJUST_ACTIVE=false;
    private boolean LONG_LAUNCHER_ADJUST_ACTIVE=false;
    private long SHORT_LAUNCHER_LAST_ADJUST_TIME=0;
    private long LONG_LAUNCHER_LAST_ADJUST_TIME=0;

    private final double LAUNCHER_ADJUST_HYSTERISIS=500;
    private final double LAUNCHER_SHORT_RANGE_ASJUSTMENT_VALUE=0.01;
    private boolean RPM_ADJUSTMENTS_ALLOWED=false;

    private final double LAUNCHER_MOTOR_TICKS_PER_REV = 28.0;





    // ---------------- STOPPER SETTINGS ----------------
    private final double GATE_CLOSED_POS = 0.65;
    private final double GATE_OPENED_POS = .94;

    // ---------------- CONTROL FLAGS ----------------
    private boolean launcherOn = false;

    public boolean intake_spitout=false;
    private boolean intakeOn = false;
    private boolean ballPusherOn = false;
    private boolean gateClosedForBall = true;

    private boolean ForceShoot_WithoutRPM=false;

    private enum ShooterState {
        IDLE, STARTING, GATE_OPEN, GATE_CLOSE, SHOOTING , FORCED_SHOOT
    }
    private ShooterState shooterState = ShooterState.IDLE;
    private long stateStartTime = 0;
    private boolean shortRangeMode = false;

    CommonCamera_18954 mCameraRef;

    private final boolean ENABLE_CAMERA_DEFINE=false;

    private double Target_RPM_Shooting=0;

    private double Current_Power_Shooting=0;
    private long Current_Power_Last_Adjusted_Time=0;
    private final long Current_Power_Adjusted_Threshold=10;
    private final long Current_Power_Initial_Adjust_Threshold=500;
    private long Current_Power_Initial_Adjust_Time=0;








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
        stopperServo.setPosition(GATE_CLOSED_POS);

        if(ENABLE_CAMERA_DEFINE) {
            mCameraRef = new CommonCamera_18954(this);
        }


        telemetry.addData("Status", "Initialized");
    }

    // ---------------- LOOP METHOD ----------------
    @Override
    public void loop() {

        Pose3D pose;
        double diff_percent=0;

        //----------------- CAMERA Feedback -----------------
        if(ENABLE_CAMERA_DEFINE) {
            pose = mCameraRef.telemetryAprilTag();
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
                    gateClosedForBall = true;
                    shortRangeMode = shortPowerShot;
                    Current_Power_Initial_Adjust_Time=System.currentTimeMillis();
                    if(shortRangeMode) {
                        Current_Power_Shooting = LAUNCHER_SHORT_RANGE_POWER;
                    }
                    else
                    {
                        Current_Power_Shooting = LAUNCHER_LONG_RANGE_POWER;
                    }
                }
                break;



            case STARTING:
                if (System.currentTimeMillis() - stateStartTime >= INITIAL_SPIN_UP_TIME) { // spin-up time
                    shooterState = ShooterState.GATE_CLOSE;
                    stateStartTime = System.currentTimeMillis();
                    gateClosedForBall = true;
                }
                break;

                //gate closed
            case GATE_CLOSE:
                gateClosedForBall = true;
                if (gamepad2.dpad_down) {
                    shooterState = ShooterState.IDLE;
                    launcherOn = false;
                    gateClosedForBall = true;
                    shortRangeMode = false;
                }
                else {
                    if (System.currentTimeMillis() - stateStartTime >= GATE_CLOSED_TIME) { // 0.5 sec open
                        shooterState = ShooterState.GATE_OPEN;
                        stateStartTime = System.currentTimeMillis();
                    }
                }
                break;

                //Gate open
            case GATE_OPEN:

                if (gamepad2.dpad_down) {
                    shooterState = ShooterState.IDLE;
                    launcherOn = false;
                    gateClosedForBall = true;
                    shortRangeMode = false;
                }
                else {
                        if (shortRangeMode) {
                            Target_RPM_Shooting = LAUNCHER_SHORTTANGE_RPM;
                        } else {
                            Target_RPM_Shooting = LAUNCHER_LONGRANGE_RPM;
                        }

                        if(gateClosedForBall && ( Math.abs(getLauncherRpm() - Target_RPM_Shooting) <= ( LAUNCHER_RPM_TOLERANCE))
                        ){
                            //open the gate
                            gateClosedForBall = false;
                            stateStartTime = System.currentTimeMillis();
                            shooterState = ShooterState.SHOOTING;
                        }
                        //Todo: Remove shortRangeMode check when we have more power
                        else if(gateClosedForBall && (ForceShoot_WithoutRPM || !shortRangeMode))
                         {
                            //open the gate
                            gateClosedForBall = false;
                            stateStartTime = System.currentTimeMillis();
                            shooterState = ShooterState.FORCED_SHOOT;
                        }
                }
                break;

            case FORCED_SHOOT:
            case SHOOTING:
            {
                if(!gateClosedForBall) {
                    if (System.currentTimeMillis() - stateStartTime >= GATE_OPEN_TIME) { // 0.5 sec close
                        shooterState = ShooterState.GATE_CLOSE;
                        stateStartTime = System.currentTimeMillis();
                        gateClosedForBall = true;
                    }
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
        if(intakeOn == true) {
            intakeMotor.setPower(1.0);
        }
        else if(intake_spitout == true) {
            intakeMotor.setPower(-0.4);
        }
        else {
            intakeMotor.setPower(0.0);
        }

        // ---------------- BALL PUSHER ----------------
        ballPusherOn = launcherOn || intakeOn ;
        if(ballPusherOn == true) {
            ballPusherMotor.setPower(1.0);
        }
        else if(intake_spitout == true) {
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

        if (launcherOn) {
            double powertoset;
            double current_launcher_rpm;

            if (shortRangeMode) {
                Target_RPM_Shooting = LAUNCHER_SHORTTANGE_RPM;
            } else {
                Target_RPM_Shooting = LAUNCHER_LONGRANGE_RPM;
            }

            current_launcher_rpm = getLauncherRpm();
            diff_percent = Math.abs(current_launcher_rpm - Target_RPM_Shooting) / Math.abs(Target_RPM_Shooting);


            if (System.currentTimeMillis() - Current_Power_Initial_Adjust_Time > Current_Power_Initial_Adjust_Threshold) {
                if (System.currentTimeMillis() - Current_Power_Last_Adjusted_Time > Current_Power_Adjusted_Threshold) {
                    if (getLauncherRpm() < Target_RPM_Shooting) {
                        Current_Power_Shooting += (LAUNCHER_POWER_ADJUSTER * diff_percent);
                        powertoset = Current_Power_Shooting;
                    } else {
                        Current_Power_Shooting -= (LAUNCHER_POWER_ADJUSTER * diff_percent);
                        powertoset = Current_Power_Shooting;
                    }
                    Current_Power_Last_Adjusted_Time = System.currentTimeMillis();
                }
            }

                if (Current_Power_Shooting > 1.0) {
                    Current_Power_Shooting = 1.0;
                } else if (Current_Power_Shooting < 0.1) {
                    Current_Power_Shooting = 0.1;
                }





			launcherMotor.setPower(Current_Power_Shooting);
            //launcherBottomMotor.setPower(Current_Power_Shooting*LAUNCHER_BACK_RATIO);
                
        } else {
                launcherMotor.setPower(0);
            //launcherBottomMotor.setPower(0);
        }


        stopperServo.setPosition(gateClosedForBall ? GATE_CLOSED_POS:GATE_OPENED_POS );
        // ---------------- TELEMETRY ----------------
        telemetry.addData("Speed Multiplier", String.format("%.2f", speedMultiplier));
        telemetry.addData("GateClosed", gateClosedForBall ? "CLOSED" : "OPEN");
        telemetry.addData("Ball Pusher", ballPusherOn ? "Running" : "Stopped");
        telemetry.addData("Launcher", launcherOn ? "Running" : "Stopped");
        telemetry.addData("Launcher Mode", shortRangeMode ? "SHORT RANGE (15%)" : "FULL POWER");
        telemetry.addData("Intake", intakeOn ? "Running" : "Stopped");
        telemetry.addData("Shooter State", shooterState.toString());
        telemetry.addData("Launcher RPM",getLauncherRpm());
        telemetry.addData("CurrentPower",Current_Power_Shooting);
        telemetry.addData("Differential",diff_percent);
        telemetry.addData("RPM Based Shooting",!ForceShoot_WithoutRPM);
        telemetry.addData("Short Range RPM",LAUNCHER_SHORTTANGE_RPM);
        telemetry.addData("Long Range RPM",LAUNCHER_LONGRANGE_RPM);
        telemetry.addData("RPM Modifiable ?",RPM_ADJUSTMENTS_ALLOWED);



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
}