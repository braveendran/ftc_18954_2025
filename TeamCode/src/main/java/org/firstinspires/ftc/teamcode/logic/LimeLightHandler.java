package org.firstinspires.ftc.teamcode.logic;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.logic.CommonDefs;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimeLightHandler {

    private final Limelight3A limelight;
    private final IMU imu;
    private Pose3D last_botpose;
    private LLResult pose_result;
    private long last_updatedtime;



    public LimeLightHandler(IMU imu, HardwareMap hardwareMap,CommonDefs.Alliance alliance) {

        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(60); // This sets how often we ask Limelight for data (100 times per second)
        if(alliance==CommonDefs.Alliance.RED) {
            limelight.pipelineSwitch(0);
        }
        else {
            limelight.pipelineSwitch(1);
        }
        limelight.start(); // This tells Limelight to start looking!
        last_botpose=null;
        update(0);
    }

    /**
     * Updates the robot's pose based on the latest Limelight data.
     * This method should be called repeatedly in a loop.
     */
    public LLResult update(long current_ms) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result !=null && result.isValid()) {

            //copy result to pose_result
            pose_result=result;
            // Get the pose of the robot with respect to the blue alliance AprilTags
            last_botpose = result.getBotpose();
            last_updatedtime=current_ms;
            return pose_result;
        }

        return null;
    }

    /**
     * Transform a Limelight Pose3D (meters, limelight coordinate frame)
     * into a field Pose in inches for use by the rest of the codebase.
     *
     * Mapping assumptions:
     * - Limelight coordinates: origin at field center (0,0). Units: meters.
     * - Field coordinate system used elsewhere: origin lower-left corner (0,0), inches,
     *   with field center at (72,72) inches and red corner at (144,144).
     * - The limelight coordinate X axis is mirrored relative to the field X axis.
     *
     * Transform implemented as:
     *   fieldX_inches = FIELD_CENTER_INCHES - limelightX_inches
     *   fieldY_inches = FIELD_CENTER_INCHES + limelightY_inches
     *   fieldHeading_deg = normalize(180 - limelightHeading_deg)
     *
     * @param botPose limelight Pose3D (meters)
     * @return Pose with x,y in inches and heading in radians (same Pose class used elsewhere)
     */
    public Pose transformBotposeToFieldInches(Pose3D botPose) {
        if (botPose == null) return null;

        // Convert limelight meters to inches using existing helpers
        double limelightX_in = CommonDefs.ConvertCameraPosToInches_x(botPose.getPosition().y);
        double limelightY_in = -CommonDefs.ConvertCameraPosToInches_y(botPose.getPosition().x);

        // Field center in inches
        final double FIELD_CENTER_IN = 72.0;

        // Apply transform: reflect X, translate origin from center to lower-left
        double fieldX = FIELD_CENTER_IN + limelightX_in;
        double fieldY = FIELD_CENTER_IN + limelightY_in;

        // Convert heading (yaw) from limelight frame to field frame (degrees)
        double limelightYawDeg = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
        double fieldYawDeg = 180.0 - limelightYawDeg;
        // Normalize to [-180,180]
        while (fieldYawDeg > 180.0) fieldYawDeg -= 360.0;
        while (fieldYawDeg <= -180.0) fieldYawDeg += 360.0;

        // Return Pose with heading in radians (consistent with existing Pose usage)
        return new Pose(fieldX, fieldY, Math.toRadians(fieldYawDeg));
    }

    public Pose3D getLast_botpose() {
        return last_botpose;
    }
    public LLResult getLast_botposeResult() {
        return pose_result;
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
