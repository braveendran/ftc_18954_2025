package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.logic.CommonDefs;

@Autonomous(name = "BlueFar", group = "Autonomous")
public class AutonomousBlueFar extends LinearOpMode {

    private AutonMovement autonMovement;

    @Override
    public void runOpMode() {
        autonMovement = new AutonMovement(this, CommonDefs.Alliance.BLUE, CommonDefs.PositionType.FAR);
        autonMovement.runAutonomousSequence();
    }
}
