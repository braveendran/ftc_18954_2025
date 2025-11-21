package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimeLightHandler {

    private final Limelight3A limelight;
    private final IMU imu;
    private Pose3D last_botpose;
    private long last_updatedtime;


    public LimeLightHandler(IMU imu, HardwareMap hardwareMap) {
        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
        limelight.start(); // This tells Limelight to start looking!
        last_botpose=null;
        update(0);
    }

    /**
     * Updates the robot's pose based on the latest Limelight data.
     * This method should be called repeatedly in a loop.
     */
    public Pose3D update(long current_ms) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Get the pose of the robot with respect to the blue alliance AprilTags
            last_botpose = result.getBotpose();
            last_updatedtime=current_ms;
            return last_botpose;
        }

        return null;
    }

    public Pose3D getLast_botpose() {
        return last_botpose;
    }

    public long getLast_updatedTime()
    {
        return last_updatedtime;
    }

    /**
     * Stops the Limelight from polling for data.
     * Call this when the OpMode is stopped.
     */
    public void stop() {
        limelight.stop();
    }
}
