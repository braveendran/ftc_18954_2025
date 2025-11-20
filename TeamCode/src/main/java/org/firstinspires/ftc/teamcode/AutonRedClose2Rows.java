package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedClose2Rows", group = "Autonomous")
public class AutonRedClose2Rows extends LinearOpMode {

    private AutonClose autonClose;

    @Override
    public void runOpMode() {
        autonClose = new AutonClose(this, AutonClose.Alliance.RED, true);
        autonClose.runAutonomousSequence();
    }
}