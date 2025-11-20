package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedClose", group = "Autonomous")
public class AutonRedClose extends LinearOpMode {

    private AutonClose autonClose;

    @Override
    public void runOpMode() {
        autonClose = new AutonClose(this, AutonClose.Alliance.RED);
        autonClose.runAutonomousSequence();
    }
}