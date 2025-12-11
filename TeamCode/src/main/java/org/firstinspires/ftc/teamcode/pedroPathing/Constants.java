package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)
            .forwardZeroPowerAcceleration(-33.83222498135969)
            .lateralZeroPowerAcceleration(-52.652509588286456)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.023))
            .headingPIDFCoefficients(new PIDFCoefficients(0.95, 0, 0.001, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6, 0, 0.001, 0.6, 0.025))
            .centripetalScaling(0.0005);




    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FrontRight")
            .rightRearMotorName("BackRight")
            .leftRearMotorName("BackLeft")
            .leftFrontMotorName("FrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(51.62987786617555)
            .yVelocity(62.05100000838557);




    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("lateralencoder")
            .strafeEncoder_HardwareMapName("ballPusherMotor")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            )
            .forwardTicksToInches(0.0020019)
            .strafeTicksToInches(-0.0019621)
            .forwardPodY(2.6)
            .strafePodX(5.35);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 1.5);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
