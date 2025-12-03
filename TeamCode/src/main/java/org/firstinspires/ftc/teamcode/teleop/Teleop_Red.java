package org.firstinspires.ftc.teamcode;

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


import org.firstinspires.ftc.teamcode.logic.Common_Teleop;



@TeleOp(name = "Teleop_Red", group = "Teleop")
public class Teleop_Red extends OpMode {

    

    private CommonDefs.Alliance mAlliance=  CommonDefs.Alliance.RED;
    private Common_Teleop mCommonTeleop;



    // ---------------- INIT METHOD ----------------
    @Override
    public void init() {

        // how to get the parent opmode refernce

       mCommonTeleop=new Common_Teleop(this,hardwareMap,telemetry,mAlliance);
       mCommonTeleop.init();
    }

    // ---------------- LOOP METHOD ----------------
    @Override
    public void loop() {

       mCommonTeleop.loop();
    }
}