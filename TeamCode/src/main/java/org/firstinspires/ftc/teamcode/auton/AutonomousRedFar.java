package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.logic.CommonDefs;

@Autonomous(name = "RedFar", group = "Autonomous")
public class AutonomousRedFar extends LinearOpMode {

    private AutonMovement autonMovement;

    @Override
    public void runOpMode() {
        autonMovement = new AutonMovement(this, CommonDefs.Alliance.RED, CommonDefs.PositionType.FAR);
        autonMovement.runAutonomousSequence();
    }
}
