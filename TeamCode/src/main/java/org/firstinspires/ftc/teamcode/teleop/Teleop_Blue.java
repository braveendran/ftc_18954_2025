package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.logic.CommonDefs;


import org.firstinspires.ftc.teamcode.logic.Common_Teleop;



@TeleOp(name = "Teleop_Blue", group = "Teleop")
public class Teleop_Blue extends OpMode {

    

    private CommonDefs.Alliance mAlliance=  CommonDefs.Alliance.BLUE;
    private Common_Teleop mCommonTeleop;



    // ---------------- INIT METHOD ----------------
    @Override
    public void init() {

        // how to get the parent opmode refernce

       mCommonTeleop=new Common_Teleop();
       mCommonTeleop.init(this,mAlliance);
    }

    // ---------------- LOOP METHOD ----------------
    @Override
    public void loop() {

       mCommonTeleop.loop();
    }
}