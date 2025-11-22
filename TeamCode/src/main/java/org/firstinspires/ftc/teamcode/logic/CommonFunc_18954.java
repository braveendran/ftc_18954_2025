package org.firstinspires.ftc.teamcode.logic;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Teleop_VelocityBased;
import org.firstinspires.ftc.teamcode.logic.DistVelocityProjection;


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

    private final double LAUNCHER_MOTOR_TICKS_PER_REV = 28.0;

    private final boolean use_relative_turn=false;


    long MINIMAL_SLEEP_TIME=1;

    IMU imu;
    double StartingYaw;
    LimeLightHandler mLimeLightHandler;
    DistVelocityProjection mDistanceDistVelocityProjection;



    public CommonFunc_18954(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;



    }

    public double getIMUYaw() {
        if(imu!=null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        else {
            return 361;
        }
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
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);
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
        stopperServo.setPosition(Teleop_VelocityBased.GATE_DOWN_PUSHED_BALL_IN_SERVOPOS);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        StartingYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        mLimeLightHandler = new LimeLightHandler(imu, hardwareMap);
        mDistanceDistVelocityProjection = new DistVelocityProjection();

        telemetry.addData("Status", "Hardware Initialized");
        telemetry.update();
    }

    public void StartShooter(long LauncherRPM, double BallPusherVelocity) {
        intakeMotor.setVelocity(3200);
        setLauncherRPM(LauncherRPM);
        //launcherMotor.setPower(LauncherRPM);
        //ballPusherMotor.setVelocity(3200);
        bShooterRunning=true;
    }
    /**
     * A self-contained function to shoot 3 Power Core.
     */
    public void shootPowerCore(long LauncherRPM,boolean unused ,double BallPusherVelocity,boolean far) {
        //telemetry.addData("Shooter", "Starting sequence...");
        //telemetry.update();

        long shooter_start_time;


        // Spin up the launcher motor
        if(!bShooterRunning) {
            StartShooter(LauncherRPM,BallPusherVelocity);
        }

        stopperServo.setPosition(Teleop_VelocityBased.GATE_DOWN_PUSHED_BALL_IN_SERVOPOS);
        opMode.sleep(400);



        for (int i=0;i<4;i++) {

            ballPusherMotor.setVelocity(3200);
            // Open the stopper to feed the Power Core
            stopperServo.setPosition(Teleop_VelocityBased.GATE_UP_RAMP_FREE_SERVOPOS_AUTON);
            if(far == true)
            {
                opMode.sleep(800);
            }
            else {
                opMode.sleep(550); // Wait 0.5 seconds for the core to pass
            }
            shooter_start_time = System.currentTimeMillis();
            while (( Math.abs (getLauncherRpm() - LauncherRPM) >= Teleop_VelocityBased.LAUNCHER_RPM_TOLERANCE) &&  (( System.currentTimeMillis() - shooter_start_time )<  Teleop_VelocityBased.MAX_WAITTIME_ACHIEVING_RPM))
            {
                opMode.sleep(1);
            }


            //ballPusherMotor.setVelocity(0);
            // Close the stopper and turn off the launcher
            stopperServo.setPosition(Teleop_VelocityBased.GATE_DOWN_PUSHED_BALL_IN_SERVOPOS);
            opMode.sleep((long)(Teleop_VelocityBased.SHOOTING_POSITION_TIME));
        }

        setLauncherRPM(0);
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

    public void turn(double speed, double relative_angle,double absolute_angle, double timeoutS) {

        if(use_relative_turn)
        {
            turn_relative(speed, relative_angle, timeoutS);
        }
        else
        {
            turn_absolute(speed, absolute_angle, timeoutS);
        }
    }

   private void turn_absolute(double speed, double absolute_yaw, double timeoutS)
   {
       // Helper lambdas for angle normalization and shortest difference
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

       // Read current heading (degrees)
       double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
       double targetYaw = norm.applyAsDouble(absolute_yaw);

       //final double HEADING_TOLERANCE_DEG = 1.5; // degrees
       final double MIN_POWER = 0.10; // minimum power to overcome static friction
       final double K_P = 0.015; // proportional gain (tune for your robot)
       long startMs = System.currentTimeMillis();

       // Ensure motors are in a mode suitable for setting power directly
       setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

       while (opMode.opModeIsActive() && ((System.currentTimeMillis() - startMs) < (long)(timeoutS * 1000))) {
           currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
           double error = shortestDiff.apply(targetYaw, currentYaw);

           // If within tolerance, we're done
           if (Math.abs(error) <= CommonDefs.HEADING_TOLERANCE_DEG) break;

           // P-controller output
           double output = K_P * error;
           double absOut = Math.min(Math.abs(output), Math.abs(speed));
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

           opMode.sleep(10);
       }

       // Stop motion
       leftFront.setPower(0);
       rightFront.setPower(0);
       leftBack.setPower(0);
       rightBack.setPower(0);

       // Restore encoder mode
       setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);



   }
    private void turn_relative(double speed, double angle, double timeoutS) {
        // Prefer IMU-based closed-loop turning, with Limelight updates while turning.
        // If IMU is not initialized, fall back to the encoder-based approximation.
        if (imu == null) {
            // Fallback: use geometric approximation based on track width
            double track = CommonDefs.WHEEL_TRACK_INCHES;
            double turnCircumference = Math.PI * track; // circumference = PI * diameter
            double turnDistanceInches = (angle / 360.0) * turnCircumference;
            encoderDrive(speed, -turnDistanceInches, turnDistanceInches, timeoutS);
            return;
        }

        // Helper lambdas for angle normalization and shortest difference
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

        // Read current heading (degrees)
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetYaw = norm.applyAsDouble(currentYaw + angle);

        final double HEADING_TOLERANCE_DEG = 1.5; // degrees
        final double MIN_POWER = 0.10; // minimum power to overcome static friction
        final double K_P = 0.015; // proportional gain (tune for your robot)
        long startMs = System.currentTimeMillis();

        // Ensure motors are in a mode suitable for setting power directly
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opMode.opModeIsActive() && ((System.currentTimeMillis() - startMs) < (long)(timeoutS * 1000))) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = shortestDiff.apply(targetYaw, currentYaw);

            // If within tolerance, we're done
            if (Math.abs(error) <= HEADING_TOLERANCE_DEG) break;

            // P-controller output
            double output = K_P * error;
            double absOut = Math.min(Math.abs(output), Math.abs(speed));
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

            opMode.sleep(10);
        }

        // Stop motion
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Restore encoder mode
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public double getLauncherRpm() {
        // First, check if launcherMotor is a DcMotorEx instance to safely call getVelocity()
        if (launcherMotor instanceof DcMotorEx) {
            // getVelocity() returns the speed in "ticks per second"
            double ticksPerSecond = ((DcMotorEx) launcherMotor).getVelocity();

            // Convert ticks per second to revolutions per minute
            // (ticks/sec) * (60 sec/min) / (ticks/rev) = rev/min
            return (ticksPerSecond * 60) / LAUNCHER_MOTOR_TICKS_PER_REV;
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
