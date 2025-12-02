package org.firstinspires.ftc.teamcode.logic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Adapter to allow CommonFunc_18954 (designed for LinearOpMode) to work with regular OpMode
 */
public class LinearOpModeAdapter extends LinearOpMode {
    private final OpMode opMode;
    
    public LinearOpModeAdapter(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }
    
    @Override
    public void runOpMode() {
        // Not used in this adapter
    }
    

//    public boolean m_opModeIsActive() {
//        return !opMode.isStopRequested();
//    }
    
//    @Override
//    public void sleep(long milliseconds) {
//        try {
//            Thread.sleep(milliseconds);
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
//    }
    

//    //public boolean m_isStopRequested() {
//        return opMode.isStopRequested();
//    }
    
//s
}