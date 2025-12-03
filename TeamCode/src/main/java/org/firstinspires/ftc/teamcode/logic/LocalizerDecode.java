package org.firstinspires.ftc.teamcode.logic;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.logic.CommonDefs.Alliance;
import com.pedropathing.geometry.Pose;
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
            // Example Pose 1: Aligned to score on the backdrop from the Red side
            validPoses.add(new ValidPose(36.0, -60.0, 3.0, -90.0, 2.0));
            // Example Pose 2: Aligned to pick up pixels from the Red wing
            validPoses.add(new ValidPose(-58.0, -36.0, 3.0, 180.0, 5.0));
        }

        // Initialize LED to a neutral/off state (gray)
        this.driverIndicationLED.off();
    }

    /**
     * This method should be called repeatedly in a loop.
     * It checks if the robot's current pose matches any of the valid poses
     * and updates the LED color to show alignment status.
     */
    public void update(long time_ms) {
        // Get the latest data from the Limelight
        Pose3D botPose =limeLightHandler.update(time_ms);

        if (botPose != null) {
            // CORRECTED: Get yaw from the rotation object
            double currentHeading = botPose.getOrientation().getYaw(AngleUnit.DEGREES);

            // The X and Y are public fields
            double currentX = CommonDefs.ConvertCameraPosToInches(botPose.getPosition().x);
            double currentY = CommonDefs.ConvertCameraPosToInches(botPose.getPosition().y);

            boolean isAligned = false;
            // Check against each valid pose in our list
            for (ValidPose validPose : validPoses) {
                // Check if heading is within tolerance
                boolean headingMatch = Math.abs(currentHeading - validPose.targetHeading) <= validPose.headingTolerance;

                // Check if position is within tolerance (using distance formula)
                double distance = Math.sqrt(Math.pow(currentX - validPose.targetX, 2) + Math.pow(currentY - validPose.targetY, 2));
                boolean positionMatch = distance <= validPose.positionTolerance;

                // If both heading and position match, we are aligned
                if (headingMatch && positionMatch) {
                    isAligned = true;
                    break; // Exit loop since we found a match
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
            // No AprilTag visible, so we are not aligned. Set LED to red.
            driverIndicationLED.setRed();
        }
    }
}