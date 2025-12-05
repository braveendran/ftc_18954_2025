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

    /**
     * Private class for combining odometry pod data with camera localization
     * to provide accurate field positioning for a 144x144 inch field.
     * Red basket at (144,144), Blue basket at (0,144).
     */
    private static class PositionLocalization {
        // Field dimensions
        private static final double FIELD_SIZE_INCHES = 144.0;
        
        // Pod configuration
        private static final double FORWARD_POD_OFFSET_X = 8.0; // 8 inches right of center
        private static final double STRAFER_POD_OFFSET_Y = 0.0; // Left corner offset
        
        // Odometry wheel specifications
        private static final double WHEEL_DIAMETER_MM = 32.0;
        private static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_MM / 25.4) * Math.PI;
        private static final double TICKS_PER_REVOLUTION = 8192.0; // Typical for high-resolution encoders
        private static final double INCHES_PER_TICK = WHEEL_CIRCUMFERENCE_INCHES / TICKS_PER_REVOLUTION;
        
        // Current position and orientation
        private double currentX = 72.0; // Start at field center
        private double currentY = 72.0; // Start at field center
        private double currentHeading = 0.0; // Degrees
        
        // Previous encoder readings
        private long previousForwardTicks = 0;
        private long previousStraferTicks = 0;
        
        // Camera fusion parameters
        private boolean cameraAvailable = false;
        private double cameraWeight = 0.3; // How much to trust camera vs odometry
        private long lastCameraUpdateTime = 0;
        private static final long CAMERA_TIMEOUT_MS = 1000; // Consider camera stale after 1 second
        
        // Pose tracking
        private double lastCameraX = 0.0;
        private double lastCameraY = 0.0;
        private double lastCameraHeading = 0.0;
        
        /**
         * Constructor initializes the localization system
         * @param startX Starting X position in inches
         * @param startY Starting Y position in inches  
         * @param startHeading Starting heading in degrees
         */
        public PositionLocalization(double startX, double startY, double startHeading) {
            this.currentX = Math.max(0, Math.min(FIELD_SIZE_INCHES, startX));
            this.currentY = Math.max(0, Math.min(FIELD_SIZE_INCHES, startY));
            this.currentHeading = normalizeAngle(startHeading);
        }
        
        /**
         * Default constructor - starts at field center
         */
        public PositionLocalization() {
            this(72.0, 72.0, 0.0);
        }
        
        /**
         * Periodic update function to recalculate robot position
         * @param forwardTicks Current encoder ticks from forward pod
         * @param straferTicks Current encoder ticks from strafer pod
         * @param cameraResult Camera localization result (can be null)
         * @param handler LimeLightHandler for camera pose extraction
         * @param currentTimeMs Current system time in milliseconds
         */
        public void periodicUpdate(long forwardTicks, long straferTicks, LLResult cameraResult, LimeLightHandler handler, long currentTimeMs) {
            // Update odometry
            updateOdometry(forwardTicks, straferTicks);
            
            // Fuse with camera data if available
            if (cameraResult != null && cameraResult.isValid()) {
                fuseCameraData(cameraResult, currentTimeMs, handler);
            } else {
                // Check if camera data is stale
                if (currentTimeMs - lastCameraUpdateTime > CAMERA_TIMEOUT_MS) {
                    cameraAvailable = false;
                }
            }
            
            // Ensure position stays within field bounds
            constrainToField();
        }
        
        /**
         * Update position based on odometry pod readings
         */
        private void updateOdometry(long forwardTicks, long straferTicks) {
            // Calculate movement deltas
            long deltaForwardTicks = forwardTicks - previousForwardTicks;
            long deltaStraferTicks = straferTicks - previousStraferTicks;
            
            double deltaForwardInches = deltaForwardTicks * INCHES_PER_TICK;
            double deltaStraferInches = deltaStraferTicks * INCHES_PER_TICK;
            
            // Convert to field coordinates considering robot orientation
            double headingRad = Math.toRadians(currentHeading);
            double cosHeading = Math.cos(headingRad);
            double sinHeading = Math.sin(headingRad);
            
            // Transform pod movements to field coordinate system
            double deltaX = deltaForwardInches * cosHeading - deltaStraferInches * sinHeading;
            double deltaY = deltaForwardInches * sinHeading + deltaStraferInches * cosHeading;
            
            // Update position
            currentX += deltaX;
            currentY += deltaY;
            
            // Update encoder readings
            previousForwardTicks = forwardTicks;
            previousStraferTicks = straferTicks;
        }
        
        /**
         * Fuse camera localization data with odometry
         */
        private void fuseCameraData(LLResult cameraResult, long currentTimeMs, LimeLightHandler handler) {
            Pose3D botPose = extractBotPose(cameraResult, handler);
            if (botPose != null) {
                // Convert camera coordinates to field coordinates
                double cameraX = CommonDefs.ConvertCameraPosToInches_x(botPose.getPosition().x);
                double cameraY = CommonDefs.ConvertCameraPosToInches_y(botPose.getPosition().y);
                double cameraHeading = normalizeAngle(botPose.getOrientation().getYaw(AngleUnit.DEGREES));
                
                // Weighted fusion of odometry and camera data
                if (cameraAvailable) {
                    currentX = currentX * (1.0 - cameraWeight) + cameraX * cameraWeight;
                    currentY = currentY * (1.0 - cameraWeight) + cameraY * cameraWeight;
                    currentHeading = blendAngles(currentHeading, cameraHeading, cameraWeight);
                } else {
                    // First camera reading - use it more heavily to correct drift
                    currentX = currentX * 0.5 + cameraX * 0.5;
                    currentY = currentY * 0.5 + cameraY * 0.5;
                    currentHeading = blendAngles(currentHeading, cameraHeading, 0.5);
                }
                
                // Update camera state
                lastCameraX = cameraX;
                lastCameraY = cameraY;
                lastCameraHeading = cameraHeading;
                lastCameraUpdateTime = currentTimeMs;
                cameraAvailable = true;
            }
        }
        
        /**
         * Extract bot pose from LLResult using the parent's LimeLightHandler
         */
        private Pose3D extractBotPose(LLResult result, LimeLightHandler handler) {
            if (result != null && result.isValid() && handler != null) {
                return handler.getLast_botpose();
            }
            return null;
        }
        
        /**
         * Blend two angles considering wraparound
         */
        private double blendAngles(double angle1, double angle2, double weight) {
            double diff = normalizeAngle(angle2 - angle1);
            return normalizeAngle(angle1 + diff * weight);
        }
        
        /**
         * Normalize angle to [-180, 180] degrees
         */
        private double normalizeAngle(double angleDegrees) {
            while (angleDegrees > 180.0) angleDegrees -= 360.0;
            while (angleDegrees <= -180.0) angleDegrees += 360.0;
            return angleDegrees;
        }
        
        /**
         * Ensure robot position stays within field boundaries
         */
        private void constrainToField() {
            currentX = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, currentX));
            currentY = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, currentY));
        }
        
        /**
         * Get current robot position
         * @return Pose object with current position and heading
         */
        public Pose getCurrentPosition() {
            return new Pose(currentX, currentY, Math.toRadians(currentHeading));
        }
        
        /**
         * Get current X position in inches
         */
        public double getCurrentX() {
            return currentX;
        }
        
        /**
         * Get current Y position in inches  
         */
        public double getCurrentY() {
            return currentY;
        }
        
        /**
         * Get current heading in degrees
         */
        public double getCurrentHeading() {
            return currentHeading;
        }
        
        /**
         * Get distance to red basket (144, 144)
         */
        public double getDistanceToRedBasket() {
            return Math.sqrt(Math.pow(144.0 - currentX, 2) + Math.pow(144.0 - currentY, 2));
        }
        
        /**
         * Get distance to blue basket (0, 144)
         */
        public double getDistanceToBlueBasket() {
            return Math.sqrt(Math.pow(currentX, 2) + Math.pow(144.0 - currentY, 2));
        }
        
        /**
         * Get angle to red basket in degrees
         */
        public double getAngleToRedBasket() {
            return Math.toDegrees(Math.atan2(144.0 - currentY, 144.0 - currentX));
        }
        
        /**
         * Get angle to blue basket in degrees
         */
        public double getAngleToBlueBasket() {
            return Math.toDegrees(Math.atan2(144.0 - currentY, 0.0 - currentX));
        }
        
        /**
         * Check if camera localization is currently available and recent
         */
        public boolean isCameraAvailable() {
            return cameraAvailable;
        }
        
        /**
         * Reset position to specified coordinates
         */
        public void resetPosition(double x, double y, double heading) {
            currentX = Math.max(0, Math.min(FIELD_SIZE_INCHES, x));
            currentY = Math.max(0, Math.min(FIELD_SIZE_INCHES, y));
            currentHeading = normalizeAngle(heading);
            cameraAvailable = false;
        }
        
        /**
         * Set camera fusion weight (0.0 = pure odometry, 1.0 = pure camera)
         */
        public void setCameraWeight(double weight) {
            this.cameraWeight = Math.max(0.0, Math.min(1.0, weight));
        }
    }
}