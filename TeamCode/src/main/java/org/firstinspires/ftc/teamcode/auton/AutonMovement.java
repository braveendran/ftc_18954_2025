package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.logic.CommonDefs;
import org.firstinspires.ftc.teamcode.logic.CommonFunc_18954;
import org.firstinspires.ftc.teamcode.logic.RobotState;
import org.firstinspires.ftc.teamcode.params.AutonCloseParams;
import org.firstinspires.ftc.teamcode.params.AutonFarParams;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.bylazar.telemetry.TelemetryManager;
import java.util.function.Supplier;
import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

private class MovementPaths {
    // This class can be expanded to include path definitions if needed
    //private Supplier<PathChain> pathChain_start_to_shoot, shoot_to_collect_row1, collect_row1_to_shoot, shoot_to_collect_row2, collect_row2_to_shoot, shoot_to_park;
    
    //create a list of path chain
    private List<Supplier<PathChain>> pathChains;
    private Follower follower;

    

    public MovementPaths(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType, CommonDefs.AutonRowsToCollect rowsToCollect, Follower follower) {
        this.follower = follower;
        // Initialize pathChains based on alliance, positionType, and rowsToCollect
        pathChains = new ArrayList<>();
        // Example: Add path chains based on the parameters
        pathChains.add(StartingPose_To_Shoot(alliance, positionType));
        pathChains.add(Shoot_To_Row1Start(alliance, positionType));
        pathChains.add(Row1Start_Row1Collect(alliance, positionType));
        pathChains.add(Row1Collect_Shoot(alliance, positionType));
        if (rowsToCollect == CommonDefs.AutonRowsToCollect.ROS_2 || rowsToCollect == CommonDefs.AutonRowsToCollect.ROS_3) {
            pathChains.add(Shoot_To_Row2Start(alliance, positionType));
            pathChains.add(Row2Start_Row2Collect(alliance, positionType));
            pathChains.add(Row2Collect_Shoot(alliance, positionType));
        }
        if(rowsToCollect == CommonDefs.AutonRowsToCollect.ROS_3) {
            pathChains.add(Shoot_To_Row3Start(alliance, positionType));
            pathChains.add(Row3Start_Row3Collect(alliance, positionType));
            pathChains.add(Row3Collect_Shoot(alliance, positionType));
        }
        pathChains.add(Shoot_To_Park(alliance, positionType));
    
    }   

    // Placeholder methods for creating path chains
    private Supplier<PathChain> StartingPose_To_Shoot(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;

        if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose= CommonDefs.BLUE_CLOSE_SHOOT_POSE;
        } else if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            targetPose= CommonDefs.BLUE_FAR_SHOOT_POSE;
        } else if(alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose= CommonDefs.RED_CLOSE_SHOOT_POSE;
        } else {
            targetPose= CommonDefs.RED_FAR_SHOOT_POSE;
        }

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    // Additional methods for other path segments would follow a similar pattern
    private Supplier<PathChain> Shoot_To_Collect_Row1(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose= CommonDefs.BLUE_ROW1_POSE;
        } else if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            targetPose= CommonDefs.BLUE_ROW3_POSE;
        } else if(alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose= CommonDefs.RED_ROW1_POSE;
        } else {
            targetPose= CommonDefs.RED_ROW3_POSE;
        }

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    private Supplier<PathChain> Row1Start_Row1Collect(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose= CommonDefs.BLUE_ROW1_COLLECT_POSE;
        } else if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            targetPose= CommonDefs.BLUE_ROW3_COLLECT_POSE;
        } else if(alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose= CommonDefs.RED_ROW1_COLLECT_POSE;
        } else {
            targetPose= CommonDefs.RED_ROW3_COLLECT_POSE;
        }

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    private Supplier<PathChain> Shoot_To_Row2Start(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.BLUE_ROW2_POSE;
        } else if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            targetPose = CommonDefs.BLUE_ROW2_POSE; // reuse or adjust as appropriate
        } else if(alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.RED_ROW2_POSE;
        } else {
            targetPose = CommonDefs.RED_ROW2_POSE;
        }

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    private Supplier<PathChain> Row2Start_Row2Collect(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.BLUE_ROW2_COLLECT_POSE;
        } else if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            targetPose = CommonDefs.BLUE_ROW2_COLLECT_POSE;
        } else if(alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.RED_ROW2_COLLECT_POSE;
        } else {
            targetPose = CommonDefs.RED_ROW2_COLLECT_POSE;
        }

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    private Supplier<PathChain> Row2Collect_Shoot(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.BLUE_CLOSE_SHOOT_POSE;
        } else if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            targetPose = CommonDefs.BLUE_FAR_SHOOT_POSE;
        } else if(alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.RED_CLOSE_SHOOT_POSE;
        } else {
            targetPose = CommonDefs.RED_FAR_SHOOT_POSE;
        }

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    private Supplier<PathChain> Shoot_To_Row3Start(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE) {
            targetPose = CommonDefs.BLUE_ROW3_POSE;
        } else {
            targetPose = CommonDefs.RED_ROW3_POSE;
        }

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    private Supplier<PathChain> Row3Start_Row3Collect(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE) {
            targetPose = CommonDefs.BLUE_ROW3_COLLECT_POSE;
        } else {
            targetPose = CommonDefs.RED_ROW3_COLLECT_POSE;
        }

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    private Supplier<PathChain> Row3Collect_Shoot(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        return Row2Collect_Shoot(alliance, positionType);
    }

    private Supplier<PathChain> Shoot_To_Park(CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
        Supplier<PathChain> pathChain;
        Pose targetPose;
        if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.BLUE_PARK_POSE;
        } else if(alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            targetPose = CommonDefs.BLUE_PARK_POSE;
        } else if(alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            targetPose = CommonDefs.RED_PARK_POSE;
        } else {
            targetPose = CommonDefs.RED_PARK_POSE;
        }

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, targetPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(targetPose.heading), 0.8))
                .build();

        return pathChain;
    }

    //Continue with other path segments similarly...
    
    
}
public class AutonMovement {
    
    private LinearOpMode opMode;
    private AutonCloseParams closeParams;
    private AutonFarParams farParams;
    private CommonFunc_18954 objCommonFunc;
    private CommonDefs.Alliance alliance;
    private CommonDefs.PositionType positionType;
    private boolean twoRowMode;

    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    private MovementPaths Paths;

    private CommonDefs.AutonState autonState;
    private final boolean UsePedroPathing_Auton = true;
    


    
    // Constructor for Close positioning
    // public AutonMovement(LinearOpMode opMode, CommonDefs.Alliance alliance, CommonDefs.PositionType positionType) {
    //     this(opMode, alliance, positionType, false);
    // }
    
    // Constructor with two-row mode option
    public AutonMovement(LinearOpMode opMode, CommonDefs.Alliance alliance, CommonDefs.PositionType positionType, CommonDefs.AutonRowsToCollect rowsToCollect) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.positionType = positionType;
        this.twoRowMode = twoRowMode;
        this.closeParams = new AutonCloseParams();
        this.farParams = new AutonFarParams();
        this.objCommonFunc = new CommonFunc_18954(opMode);

        follower = Constants.createFollower(opMode.hardwareMap);

        Paths = new MovementPaths(alliance, positionType, rowsToCollect, follower);
        this.autonState = CommonDefs.AutonState.AUTON_MOVE_TO_SHOOT;
    }

    public void runAutonSequence_PedroPathing() {
        // Initialize starting pose based on alliance and position type
        if (alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.CLOSE) {
            startingPose = CommonDefs.BLUE_CLOSE_START_POSE;
        } else if (alliance == CommonDefs.Alliance.BLUE && positionType == CommonDefs.PositionType.FAR) {
            startingPose = CommonDefs.BLUE_FAR_START_POSE;
        } else if (alliance == CommonDefs.Alliance.RED && positionType == CommonDefs.PositionType.CLOSE) {
            startingPose = CommonDefs.RED_CLOSE_START_POSE;
        } else {
            startingPose = CommonDefs.RED_FAR_START_POSE;
        }

        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Wait for start
        opMode.waitForStart();

        // Follow each path segment in sequence
        for (Supplier<PathChain> pathChainSupplier : Paths.getPathChains()) {
            follower.followPath(pathChainSupplier.get());
            while (follower.isBusy() && opMode.opModeIsActive()) {
                follower.update();
                telemetryM.update();
            }

            if(this.autonState == CommonDefs.AutonState.AUTON_MOVE_TO_SHOOT) {
                this.autonState = CommonDefs.AutonState.AUTON_SHOOT;
                // Start shooting sequence - 3 balls, determine RPM and wait time based on position
                long shootingRPM = (positionType == CommonDefs.PositionType.CLOSE) ? closeParams.LAUNCHER_POS1_RPM : farParams.LAUNCHER_POS1_RPM;
                long waitTime = (positionType == CommonDefs.PositionType.FAR) ? 1200 : 550;
                objCommonFunc.startShootingSequence(3, shootingRPM, closeParams.BALLPUSHER_MAX_VELOCITY, waitTime);
            } else if(this.autonState == CommonDefs.AutonState.AUTON_SHOOT) {
                // Update shooting state machine
                if (objCommonFunc.updateShootingSequence()) {
                    this.autonState = CommonDefs.AutonState.AUTON_MOVE_TO_COLLECT;
                }
            } else if(this.autonState == CommonDefs.AutonState.AUTON_MOVE_TO_COLLECT) {
                // Collection logic here
                objCommonFunc.TurnOnIntake();
                this.autonState = CommonDefs.AutonState.AUTON_COLLECT_BALLS;
            }
            else if(this.autonState == CommonDefs.AutonState.AUTON_COLLECT_BALLS) {
                //objCommonFunc.TurnOffIntake();
                this.autonState = CommonDefs.AutonState.AUTON_MOVE_TO_SHOOT;
            }
            
        }
        
        // Save final pose for TeleOp continuation
        RobotState.setLastAutonPose(follower.getPose());
    }
    
    public void runAutonomousSequence() {

        if(UsePedroPathing_Auton) {
            runAutonSequence_PedroPathing();
        } else {
            if (positionType == CommonDefs.PositionType.CLOSE) {
                runCloseAutonomousSequence();
            } else {
            runFarAutonomousSequence();
            }
        }
    }
    
    private void runCloseAutonomousSequence() {
        // ---------------- INIT & HARDWARE MAPPING ----------------
        objCommonFunc.initializeHardware();
        
        // ---------------- VISION INITIALIZATION (Placeholder) ----------------
        // In a real robot, you would initialize your camera and TensorFlow/AprilTag pipeline here.
        // This function simulates detecting a signal cone.
        //detectSignalCone();
        
        // This command waits for the driver to press the START button.
        opMode.waitForStart();
        
        // ---------------- AUTONOMOUS SEQUENCE STARTS HERE ----------------
        opMode.telemetry.addData("Starting IMU", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
        // Step 1: Move forward away from the wall to get clearance for shooting
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, closeParams.INITIAL_BACKWARD_DIST, closeParams.INITIAL_BACKWARD_DIST, closeParams.DRIVE_TIMEOUT_SHORT);
        opMode.telemetry.addData("Step 1", objCommonFunc.getIMUYaw());
        // Step 2: Turn towards the high goal (more conservative angle since we're close)
        //        if (alliance == CommonDefs.Alliance.BLUE) {
        //            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.INITIAL_TURN_ANGLE, closeParams.TURN_TIMEOUT);
        //        } else { // RED
        //            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.INITIAL_TURN_ANGLE, closeParams.TURN_TIMEOUT);
        //        }
        
        // Step 3: Shoot first Power Core into the high goal  
        objCommonFunc.shootPowerCore(1, closeParams.LAUNCHER_POS1_RPM, closeParams.BALLPUSHER_MAX_VELOCITY, 550);
        opMode.telemetry.addData("Step 3", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 4: Navigate to first row collection position
        if (alliance == CommonDefs.Alliance.RED) {
            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT,closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT,-closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
        }
        opMode.telemetry.addData("Step 4", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 5: Turn on intake and collect from first row
        objCommonFunc.TurnOnIntake(closeParams.INTAKE_MAX_VELOCITY, closeParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_INTAKE, closeParams.COLLECTION_DISTANCE_ROW1, closeParams.COLLECTION_DISTANCE_ROW1, closeParams.DRIVE_TIMEOUT_LONG);
        objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, -closeParams.COLLECTION_DISTANCE_ROW1, -closeParams.COLLECTION_DISTANCE_ROW1, closeParams.DRIVE_TIMEOUT_SHORT);

        opMode.telemetry.addData("Step 5", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();

        // Step 6: Return to shooting position for second shot
        if (alliance == CommonDefs.Alliance.RED) {
            objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(closeParams.TURN_SPEED, -(closeParams.TURN_TO_COLLECT + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA),0, closeParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW1, closeParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT+ closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA,0, closeParams.TURN_TIMEOUT);
        }
        opMode.telemetry.addData("Step 6", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 7: Second shooting sequence
        objCommonFunc.shootPowerCore(1, closeParams.LAUNCHER_POS1_RPM, closeParams.BALLPUSHER_MAX_VELOCITY, 550);
        objCommonFunc.TurnOffIntake();

        opMode.telemetry.addData("Step 7", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Step 8: Move to second row collection
            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT, closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2 , closeParams.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT, - closeParams.TURN_TO_COLLECT,closeParams.TURN_TIMEOUT);
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.DIST_ROW2, closeParams.STRAFE_TIMEOUT);
            }
            opMode.telemetry.addData("Step 8", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();
            
            // Step 9: Collect from second row
            objCommonFunc.TurnOnIntake(closeParams.INTAKE_MAX_VELOCITY, closeParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_INTAKE, closeParams.COLLECTION_DISTANCE_ROW2, closeParams.COLLECTION_DISTANCE_ROW2, closeParams.DRIVE_TIMEOUT_LONG);
            objCommonFunc.encoderDrive(closeParams.DRIVE_SPEED_SLOW, -closeParams.COLLECTION_DISTANCE_ROW2, -closeParams.COLLECTION_DISTANCE_ROW2, closeParams.DRIVE_TIMEOUT_SHORT);

            opMode.telemetry.addData("Step 9", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();
            // Step 10: Return to shooting position for third shot
            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_ROW2_HIGHSPEED, closeParams.DIST_ROW2 +closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);
                objCommonFunc.turn(closeParams.TURN_SPEED, - (closeParams.TURN_TO_SHOOT_ROW2 + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA), closeParams.TURN_TO_SHOOT_ROW2_ABSOLUTE,closeParams.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_ROW2_HIGHSPEED, closeParams.DIST_ROW2 +closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);
                objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_SHOOT_ROW2 + closeParams.TURN_ANTI_CLOCKWISE_ERROR_DELTA, -closeParams.TURN_TO_SHOOT_ROW2_ABSOLUTE,closeParams.TURN_TIMEOUT);
            }
            opMode.telemetry.addData("Step 10", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();
            // Step 11: Third shooting sequence
            //objCommonFunc.shootPowerCore(closeParams.LAUNCHER_POS1_RPM, false, closeParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.TurnOffIntake();
        }
        else {

            // END OF Auton - Go to parking
            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.turn(closeParams.TURN_SPEED, closeParams.TURN_TO_COLLECT,closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(closeParams.TURN_SPEED, -closeParams.TURN_TO_COLLECT,-closeParams.TURN_TO_COLLECT, closeParams.TURN_TIMEOUT);
            }

            if (alliance == CommonDefs.Alliance.RED) {
                objCommonFunc.strafe_left(closeParams.DRIVE_SPEED_SLOW, closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);

            } else { // RED
                objCommonFunc.strafe_right(closeParams.DRIVE_SPEED_SLOW, closeParams.PARKING_DISTANCE, closeParams.STRAFE_TIMEOUT);

            }
        }
//
//
//

        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        
        // Save final pose for TeleOp continuation (encoder-based auton)
        if (!UsePedroPathing_Auton && follower != null) {
            RobotState.setLastAutonPose(follower.getPose());
        }
        
        opMode.sleep((int)closeParams.SLEEP_TIME);
    }
    
    private void runFarAutonomousSequence() {
        // ---------------- INIT & HARDWARE MAPPING ----------------
        objCommonFunc.initializeHardware();
        
        // ---------------- VISION INITIALIZATION (Placeholder) ----------------
        // In a real robot, you would initialize your camera and TensorFlow/AprilTag pipeline here.
        // This function simulates detecting a signal cone.
        //detectSignalCone();
        
        // This command waits for the driver to press the START button.
        opMode.waitForStart();
        
        // ---------------- AUTONOMOUS SEQUENCE STARTS HERE ----------------

        opMode.telemetry.addData("Starting IMU", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 1: Move forward away from the wall to get clearance for shooting
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, farParams.INITIAL_FORWARD_DISTANCE, farParams.INITIAL_FORWARD_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);

        opMode.telemetry.addData("Step 1", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();

        // Step 2: Turn towards the high goal
        if (alliance == CommonDefs.Alliance.BLUE) {
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.INITIAL_TURN_ANGLE,farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.INITIAL_TURN_ANGLE,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
        }

        opMode.telemetry.addData("Step 2", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();

        // Step 3: Shoot first Power Core into the high goal
        objCommonFunc.shootPowerCore(1, farParams.LAUNCHER_POS1_RPM, farParams.BALLPUSHER_MAX_VELOCITY, 1200);
        opMode.telemetry.addData("Step 3", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        // Step 4: Turn to collection position
        if (alliance == CommonDefs.Alliance.BLUE) {
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
            objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
        } else { // RED
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT,-farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
            objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1, farParams.STRAFE_TIMEOUT);
        }
        opMode.telemetry.addData("Step 4", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        // Step 5: Turn on intake and collect from first row
        objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_INTAKE, farParams.COLLECTION_DISTANCE, farParams.COLLECTION_DISTANCE, farParams.DRIVE_TIMEOUT_LONG);
        objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, -farParams.COLLECTION_DISTANCE, -farParams.COLLECTION_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);

        opMode.telemetry.addData("Step 5", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        // Step 6: Return to shooting position for second shot
        if (alliance == CommonDefs.Alliance.BLUE) {
            objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1 + farParams.DIST_ROW1_ADDITIONAL_RETURN, farParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT,farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
        } else { // RED
            objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW1+ farParams.DIST_ROW1_ADDITIONAL_RETURN, farParams.STRAFE_TIMEOUT);
            objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
        }
        opMode.telemetry.addData("Step 6", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Step 7: Second shooting sequence
        objCommonFunc.shootPowerCore(1, farParams.LAUNCHER_POS1_RPM, farParams.BALLPUSHER_MAX_VELOCITY, 1200);
        objCommonFunc.TurnOffIntake();
        opMode.telemetry.addData("Step 7", objCommonFunc.getIMUYaw());
        opMode.telemetry.update();
        
        // Second row logic (only if twoRowMode is enabled)
        if (twoRowMode) {
            // Step 8: Move to second row collection
            if (alliance == CommonDefs.Alliance.BLUE) {
                objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
                objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT,-farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
                objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
            }
            opMode.telemetry.addData("Step 8", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();

            
            // Step 9: Collect from second row
            objCommonFunc.TurnOnIntake(farParams.INTAKE_MAX_VELOCITY, farParams.BALLPUSHER_MAX_VELOCITY);
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_INTAKE, farParams.COLLECTION_DISTANCE_ROW2, farParams.COLLECTION_DISTANCE_ROW2, farParams.DRIVE_TIMEOUT_LONG);
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, -farParams.COLLECTION_DISTANCE_ROW2, -farParams.COLLECTION_DISTANCE_ROW2, farParams.DRIVE_TIMEOUT_SHORT);

            opMode.telemetry.addData("Step 9", objCommonFunc.getIMUYaw());
            opMode.telemetry.update();

//            // Step 10: Return to shooting position for third shot
//            if (alliance == CommonDefs.Alliance.BLUE) {
//                objCommonFunc.strafe_left(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
//                objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT,farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
//            } else { // RED
//                objCommonFunc.strafe_right(farParams.DRIVE_SPEED_SLOW, farParams.DIST_ROW2, farParams.STRAFE_TIMEOUT);
//                objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT,-farParams.INITIAL_TURN_ANGLE, farParams.TURN_TIMEOUT);
//            }
//
//            opMode.telemetry.addData("Step 10", objCommonFunc.getIMUYaw());
//            opMode.telemetry.update();
//
//            // Step 11: Third shooting sequence
//            objCommonFunc.shootPowerCore(farParams.LAUNCHER_POS1_RPM, false, farParams.BALLPUSHER_MAX_VELOCITY);
//            objCommonFunc.TurnOffIntake();
        }
        else {

            // END OF Auton - Go to parking
            if (alliance == CommonDefs.Alliance.BLUE) {
                objCommonFunc.turn(farParams.TURN_SPEED, farParams.TURN_TO_COLLECT, farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
            } else { // RED
                objCommonFunc.turn(farParams.TURN_SPEED, -farParams.TURN_TO_COLLECT, -farParams.TURN_TO_COLLECT_ABSOLUTE, farParams.TURN_TIMEOUT);
            }

            // Move out of the shooting area for parking
            objCommonFunc.encoderDrive(farParams.DRIVE_SPEED_SLOW, farParams.PARKING_DISTANCE, farParams.PARKING_DISTANCE, farParams.DRIVE_TIMEOUT_SHORT);
        }
        // Completion telemetry
        opMode.telemetry.addData("Autonomous", "Complete");
        opMode.telemetry.update();
        
        // Save final pose for TeleOp continuation (encoder-based auton)
        if (!UsePedroPathing_Auton && follower != null) {
            RobotState.setLastAutonPose(follower.getPose());
        }
        
        opMode.sleep((int)farParams.SLEEP_TIME);
    }
}