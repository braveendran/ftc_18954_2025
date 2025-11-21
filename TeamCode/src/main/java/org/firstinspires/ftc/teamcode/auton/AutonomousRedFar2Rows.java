package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.logic.CommonDefs;

@Autonomous(name = "RedFar2Rows", group = "Autonomous")
public class AutonomousRedFar2Rows extends LinearOpMode {

    private AutonMovement autonMovement;

    @Override
    public void runOpMode() {
        autonMovement = new AutonMovement(this, CommonDefs.Alliance.RED, CommonDefs.PositionType.FAR, true);
        autonMovement.runAutonomousSequence();
    }
}
