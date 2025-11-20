package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueFar2Rows", group = "Autonomous")
public class AutonomousBlueFar2Rows extends LinearOpMode {

    private AutonFar autonFar;

    @Override
    public void runOpMode() {
        autonFar = new AutonFar(this, AutonFar.Alliance.BLUE, true);
        autonFar.runAutonomousSequence();
    }
}
