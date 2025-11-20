package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedFar", group = "Autonomous")
public class AutonomousRedFar extends LinearOpMode {

    private AutonMovement autonMovement;

    @Override
    public void runOpMode() {
        autonMovement = new AutonMovement(this, AutonMovement.Alliance.RED, AutonMovement.PositionType.FAR);
        autonMovement.runAutonomousSequence();
    }
}
