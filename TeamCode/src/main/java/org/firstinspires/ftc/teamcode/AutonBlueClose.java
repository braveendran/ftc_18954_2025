package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueClose", group = "Autonomous")
public class AutonBlueClose extends LinearOpMode {

    private AutonClose autonClose;

    @Override
    public void runOpMode() {
        autonClose = new AutonClose(this, AutonClose.Alliance.BLUE);
        autonClose.runAutonomousSequence();
    }
}