package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedFar2Rows", group = "Autonomous")
public class AutonomousRedFar2Rows extends LinearOpMode {

    private AutonFar autonFar;

    @Override
    public void runOpMode() {
        autonFar = new AutonFar(this, AutonFar.Alliance.RED, true);
        autonFar.runAutonomousSequence();
    }
}
