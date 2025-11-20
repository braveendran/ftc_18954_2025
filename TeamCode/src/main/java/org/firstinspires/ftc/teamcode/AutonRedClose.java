package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedClose", group = "Autonomous")
public class AutonRedClose extends LinearOpMode {

    private AutonMovement autonMovement;

    @Override
    public void runOpMode() {
        autonMovement = new AutonMovement(this, AutonMovement.Alliance.RED, AutonMovement.PositionType.CLOSE);
        autonMovement.runAutonomousSequence();
    }
}