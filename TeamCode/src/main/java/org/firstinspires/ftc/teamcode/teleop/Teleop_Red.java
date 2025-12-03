package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.logic.CommonDefs;


import org.firstinspires.ftc.teamcode.logic.Common_Teleop;



@TeleOp(name = "Teleop_Red", group = "Teleop")
public class Teleop_Red extends OpMode {

    

    private CommonDefs.Alliance mAlliance=  CommonDefs.Alliance.RED;
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