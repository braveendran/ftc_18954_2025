package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import android.util.Size;

import java.util.List;


public class CommonCamera_18954 {

    private static final int RED_TAG_ID = 24;
    private static final int BLUE_TAG_ID = 20;

    private Position Coordinates_RedCenter = new Position(DistanceUnit.INCH,-58.3727, 55.6425, 29.5,0);
    private Position Coordinates_BlueCenter = new Position (DistanceUnit.INCH,-58.3727, -55.6425, 29.5,0);


    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private OpMode mOpeModeRef;

    public CommonCamera_18954(OpMode mOpeModeRef) {
        this.mOpeModeRef=mOpeModeRef;
        initAprilTag();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(mOpeModeRef.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    public Pose3D telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        this.mOpeModeRef.telemetry.addData("# AprilTags Detected", currentDetections.size());
        Pose3D pose=null;


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    //pose = detection.robotPose;

                    // Find the correct tag for the alliance

                    if ((detection.id == BLUE_TAG_ID) ) {
                        pose = detection.robotPose;
                    }
                    if ((detection.id == RED_TAG_ID)) {
                        pose = detection.robotPose;
                    }

                }
            }
        }   // end for() loop

        return pose;
    }   // end method telemetryAprilTag()


    public void GetMoveInstructions(boolean IsBlueTeam , CommonFunc_18954 robotFunc) {
        Pose3D pose = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        this.mOpeModeRef.telemetry.addData("# AprilTags Detected", currentDetections.size());

        Position targetCoordinates = null;
        if (IsBlueTeam) {
            targetCoordinates = Coordinates_BlueCenter;
        } else {
            targetCoordinates = Coordinates_RedCenter;
        }

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {                    // Find the correct tag for the alliance

                    if ((detection.id == BLUE_TAG_ID)) {
                        pose = detection.robotPose;
                    }
                    if ((detection.id == RED_TAG_ID)) {
                        pose = detection.robotPose;
                    }

                }
            }
        }

        if (pose != null) {
            Position tagPosition = pose.getPosition();
            double deltaX = targetCoordinates.x - tagPosition.x;
            double deltaY = targetCoordinates.y - tagPosition.y;
            double deltaZ = targetCoordinates.z - tagPosition.z;

            this.mOpeModeRef.telemetry.addData("Delta X (in): ", "%.2f", deltaX);
            this.mOpeModeRef.telemetry.addData("Delta Y (in): ", "%.2f", deltaY);
            this.mOpeModeRef.telemetry.addData("Delta Z (in): ", "%.2f", deltaZ);
        } else {
            this.mOpeModeRef.telemetry.addData("No valid tag detected for team ", IsBlueTeam ? "Blue" : "Red");
        }
    }


        
}





