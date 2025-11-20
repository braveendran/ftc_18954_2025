package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BlueClose", group = "Autonomous")
public class AutonBlueClose extends LinearOpMode {

    private AutonMovement autonMovement;

    @Override
    public void runOpMode() {
        autonMovement = new AutonMovement(this, AutonMovement.Alliance.BLUE, AutonMovement.PositionType.CLOSE);
        autonMovement.runAutonomousSequence();
    }
}