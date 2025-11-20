package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueFar2Rows", group = "Autonomous")
public class AutonomousBlueFar2Rows extends LinearOpMode {

    private AutonMovement autonMovement;

    @Override
    public void runOpMode() {
        autonMovement = new AutonMovement(this, CommonDefs.Alliance.BLUE, CommonDefs.PositionType.FAR, true);
        autonMovement.runAutonomousSequence();
    }
}
