package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueFar", group = "Autonomous")
public class AutonomousBlueFar extends LinearOpMode {

    private AutonFar autonFar;

    @Override
    public void runOpMode() {
        autonFar = new AutonFar(this, AutonFar.Alliance.BLUE);
        autonFar.runAutonomousSequence();
    }
}
