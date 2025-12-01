package org.firstinspires.ftc.teamcode.logic;

import com.pedropathing.geometry.Pose;

/**
 * Static holder for robot state that persists between OpModes
 * Used to transfer pose from Autonomous to TeleOp
 */
public final class RobotState {
    private static volatile Pose lastAutonPose = null;
    
    public static synchronized void setLastAutonPose(Pose pose) {
        lastAutonPose = pose;
    }
    
    public static synchronized Pose getLastAutonPose() {
        return lastAutonPose;
    }
    
    public static synchronized boolean hasAutonPose() {
        return lastAutonPose != null;
    }
    
    public static synchronized void clearAutonPose() {
        lastAutonPose = null;
    }
}