package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedFar", group = "Autonomous")
public class AutonomousRedFar extends LinearOpMode {

    private AutonFar autonFar;

    @Override
    public void runOpMode() {
        autonFar = new AutonFar(this, AutonFar.Alliance.RED);
        autonFar.runAutonomousSequence();
    }
}
