package org.firstinspires.ftc.teamcode.logic;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.logic.CommonDefs.Alliance;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;


/**
 * The LocalizerDecode class provides visual feedback for robot alignment.
 * It uses a Limelight to determine the robot's pose and an LED to indicate
 * if the robot is at one of the predefined valid positions and orientations.
 */
public class LocalizerDecode {

    private final LimeLightHandler limeLightHandler;
    private final DriverIndicationLED driverIndicationLED;
    private final Alliance alliance;


    // Use a list of valid poses instead of a single target
    private final List<ValidPose> validPoses;

    private double HeadingCorrectionDeg = 0.0; // degrees

    /**
     * A private inner class to hold the definition of a valid pose,
     * including position, heading, and their respective tolerances.
     */
    private static class ValidPose {
        final double targetX;
        final double targetY;
        final double positionTolerance;
        final double targetHeading;
        final double headingTolerance;

        ValidPose(double x, double y, double posTolerance, double heading, double headTolerance) {
            this.targetX = x;
            this.targetY = y;
            this.positionTolerance = posTolerance;
            this.targetHeading = heading;
            this.headingTolerance = headTolerance;
        }
    }

    /**
     * Constructor for LocalizerDecode.
     * @param alliance The alliance color (RED or BLUE).
     * @param limeLightHandler An initialized LimeLightHandler instance.
     * @param driverIndicationLED An initialized DriverIndicationLED instance.
     */
    public LocalizerDecode(Alliance alliance, LimeLightHandler limeLightHandler, DriverIndicationLED driverIndicationLED) {
        this.alliance = alliance;
        this.limeLightHandler = limeLightHandler;
        this.driverIndicationLED = driverIndicationLED;
        this.validPoses = new ArrayList<>();

        // Populate the list of valid poses based on the alliance.
        // These are placeholder values and should be tuned for your specific needs.
        if (alliance == Alliance.BLUE) {
            // Example Pose 1: Aligned to score on the backdrop from the Blue side
            validPoses.add(new ValidPose(36.0, 60.0, 3.0, 90.0, 2.0));
            // Example Pose 2: Aligned to pick up pixels from the Blue wing
            validPoses.add(new ValidPose(-58.0, 36.0, 3.0, 180.0, 5.0));
        } else { // RED
            // Close Poses
            validPoses.add(new ValidPose(-96, 66.0, 3.0, 1.0, 1.0));
            validPoses.add(new ValidPose(-95.3, 74.6, 3.0, -9.91, 1.0));
            validPoses.add(new ValidPose(-90.7, 81, 3.0, -20.57, 1.0));
            validPoses.add(new ValidPose(-85.2, 89, 3.0, -30.03, 1.0));

            //Far Pose
            validPoses.add(new ValidPose(-132, 40.3, 3.0, 20.88, 1.0));
        }

        // Initialize LED to a neutral/off state (gray)
        this.driverIndicationLED.off();
    }

    /**
     * This method should be called repeatedly in a loop.
     * It checks if the robot's current pose matches any of the valid poses
     * and updates the LED color to show alignment status.
     */
    public LLResult update(long time_ms) {
        // Get the latest data from the Limelight
        LLResult resultPose =limeLightHandler.update(time_ms);
        Pose3D botPose = null;

        boolean redFound = false;
        boolean blueFound = false;



        if (resultPose != null && resultPose.isValid()) {
            botPose = limeLightHandler.getLast_botpose();
            List<LLResultTypes.DetectorResult> detections = resultPose.getDetectorResults();
            double deltaHeading = resultPose.getTx();
            double ta= resultPose.getTa();

            boolean isAligned = false;
            // Check against each valid pose in our list
            {

                boolean headingMatch =false ;

                if(ta > CommonDefs.LIMELIGHT_HEADING_TARGETAREA_THRESHOLD ) {
                    HeadingCorrectionDeg = deltaHeading - CommonDefs.LIMELIGHT_HEADING_SHOOT_CLOSE_HEADING;
                    HeadingCorrectionDeg =HeadingCorrectionDeg*-1;
                    //Close shooting
                    headingMatch = Math.abs(deltaHeading - CommonDefs.LIMELIGHT_HEADING_SHOOT_CLOSE_HEADING) <= CommonDefs.LIMELIGHT_HEADING_SHOOT_TOLERANCE_CLOSE;
                }
                else
                {
                    HeadingCorrectionDeg =deltaHeading - CommonDefs.LIMELIGHT_HEADING_SHOOT_FAR_HEADING;
                    HeadingCorrectionDeg =HeadingCorrectionDeg*-1;
                    //Far shooting
                    headingMatch = Math.abs(deltaHeading - CommonDefs.LIMELIGHT_HEADING_SHOOT_FAR_HEADING) <= CommonDefs.LIMELIGHT_HEADING_SHOOT_TOLERANCE_FAR;
                }

                // Check if position is within tolerance (using distance formula)

                boolean positionMatch = true;

                if(CommonDefs.LOCALIZER_CHECK_DISTANCE_MATCH)
                {
                    // The X and Y are public fields
                    double currentX = CommonDefs.ConvertCameraPosToInches_x(botPose.getPosition().x);
                    double currentY = CommonDefs.ConvertCameraPosToInches_y(botPose.getPosition().y);

                    positionMatch=false;
                    for (ValidPose validPose : validPoses) {
                        double distance = Math.sqrt(Math.pow(currentX - validPose.targetX, 2) + Math.pow(currentY - validPose.targetY, 2));
                        positionMatch = distance <= validPose.positionTolerance;
                        if(positionMatch) {
                            break;
                        }
                    }
                }

                // If both heading and position match, we are aligned
                if (headingMatch && positionMatch) {
                    isAligned = true;
                }
            }

            if (isAligned) {
                // We are aligned with one of the valid poses! Set LED to green.
                driverIndicationLED.setGreen();
            } else {
                // Not aligned with any valid pose. Set LED to red.
                driverIndicationLED.setRed();
            }
        } else {
            // No AprilTag visible, so we are not aligned. Set LED to blue.
            driverIndicationLED.setBlue();
        }

        return resultPose;
    }

    public double getHeadingCorrectionDeg() {
        return HeadingCorrectionDeg;
    }
}