package org.firstinspires.ftc.teamcode.logic;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.logic.CommonDefs.Alliance;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    
    // Encoder references for odometry pods
    private final DcMotorEx lateralEncoder;    // Forward pod
    private final DcMotorEx ballPusherMotor;   // Strafer pod
    
    // Position localization instance
    private final PositionLocalization positionLocalizer;

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
     * @param forwardpod The lateral encoder motor (forward pod).
     * @param starferpod The ball pusher motor (strafer pod).
     */
    public LocalizerDecode(Alliance alliance, LimeLightHandler limeLightHandler, DriverIndicationLED driverIndicationLED,
                           DcMotorEx forwardpod, DcMotorEx starferpod) {
        this.alliance = alliance;
        this.limeLightHandler = limeLightHandler;
        this.driverIndicationLED = driverIndicationLED;
        this.lateralEncoder = forwardpod;
        this.ballPusherMotor = starferpod;
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

        // Initialize position localization system
        this.positionLocalizer = new PositionLocalization();
        
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
        LLResult resultPose = limeLightHandler.update(time_ms);
        
        // Read encoder ticks for position localization
        long forwardTicks = lateralEncoder.getCurrentPosition();
        long straferTicks = ballPusherMotor.getCurrentPosition();
        
        // Update position localization
        positionLocalizer.periodicUpdate(forwardTicks, straferTicks, resultPose, limeLightHandler, time_ms);
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
                        // Use LimeLightHandler's transform to get field coordinates in inches
                        Pose transformed = limeLightHandler.transformBotposeToFieldInches(botPose);
                        double currentX = transformed.getX();
                        double currentY = transformed.getY();

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
                // Check if we're too close to target for safe shooting
                Pose fusedPos = positionLocalizer.getCurrentFusedPosition();
                double distanceToTarget = (alliance == Alliance.RED) ? 
                    positionLocalizer.getDistanceToRedBasket() : positionLocalizer.getDistanceToBlueBasket();
                
                if (distanceToTarget < CommonDefs.TARGET_TOO_CLOSE_THRESHOLD_INCHES) {
                    // Too close - blink the LED to warn driver
                    driverIndicationLED.setBlinking(true);
                    driverIndicationLED.setGreen();
                } else {
                    // Good distance - solid green
                    driverIndicationLED.setBlinking(false);
                    driverIndicationLED.setGreen();
                }
            } else {
                Pose fusedPos = positionLocalizer.getCurrentFusedPosition();
                double distanceToTarget = (alliance == Alliance.RED) ? 
                    positionLocalizer.getDistanceToRedBasket() : positionLocalizer.getDistanceToBlueBasket();
                
                if (distanceToTarget < CommonDefs.TARGET_TOO_CLOSE_THRESHOLD_INCHES) {
                    // Too close - blink the LED to warn driver
                    driverIndicationLED.setBlinking(true);
                    driverIndicationLED.setRed();
                } else {
                    // Good distance - solid green
                    driverIndicationLED.setBlinking(false);
                    driverIndicationLED.setRed();
                }
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
     * Get the position localization instance for accessing positioning data
     */
    public PositionLocalization getPositionLocalizer() {
        return positionLocalizer;
    }

    public double getDistanceToRedBasket(){
        return positionLocalizer.getDistanceToRedBasket();
    }

    public double getDistanceToBlueBasket(){
        return positionLocalizer.getDistanceToBlueBasket();
    }
    
    /**
     * Get current fused position (combination of camera and odometry)
     */
    public Pose getCurrentFusedPosition() {
        return positionLocalizer.getCurrentFusedPosition();
    }
    
    /**
     * Get current camera-only position
     */
    public Pose getCurrentCameraPosition() {
        return positionLocalizer.getCurrentCameraPosition();
    }
    
    /**
     * Get current odometry-only position
     */
    public Pose getCurrentOdometryPosition() {
        return positionLocalizer.getCurrentOdometryPosition();
    }
    
    /**
     * Get formatted string for fused position
     */
    public String getFusedPositionString() {
        Pose fusedPos = positionLocalizer.getCurrentFusedPosition();
        return String.format("x:%.2f y:%.2f h:%.2f", 
            fusedPos.getX(), fusedPos.getY(), Math.toDegrees(fusedPos.getHeading()));
    }
    
    /**
     * Get formatted string for camera-only position
     */
    public String getCameraPositionString() {
        Pose cameraPos = positionLocalizer.getCurrentCameraPosition();
        String status = positionLocalizer.isCameraAvailable() ? "" : "(stale)";
        return String.format("x:%.2f y:%.2f h:%.2f%s", 
            cameraPos.getX(), cameraPos.getY(), Math.toDegrees(cameraPos.getHeading()), status);
    }
    
    /**
     * Get formatted string for encoder-only position
     */
    public String getEncoderPositionString() {
        if (!positionLocalizer.isOdometryInitialized()) {
            return "x:-- y:-- h:-- (uninitialized)";
        }
        Pose encoderPos = positionLocalizer.getCurrentOdometryPosition();
        return String.format("x:%.2f y:%.2f h:%.2f", 
            encoderPos.getX(), encoderPos.getY(), Math.toDegrees(encoderPos.getHeading()));
    }

    /**
     * Private class for combining odometry pod data with camera localization
     * to provide accurate field positioning for a 144x144 inch field.
     * Red basket at (144,144), Blue basket at (0,144).
     */
    private class PositionLocalization {
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
        
        // Fused position (combination of camera and odometry)
        private double fusedX = 72.0; // Start at field center
        private double fusedY = 72.0; // Start at field center
        private double fusedHeading = 0.0; // Degrees
        
        // Camera-only position
        private double cameraX = 72.0;
        private double cameraY = 72.0;
        private double cameraHeading = 0.0;
        
        // Odometry-only position
        private double odometryX = 72.0;
        private double odometryY = 72.0;
        private double odometryHeading = 0.0;
        private boolean odometryInitialized = false; // Track if odometry has been initialized
        
        // Previous encoder readings
        private long previousForwardTicks = 0;
        private long previousStraferTicks = 0;
        
        // Camera fusion parameters
        private boolean cameraAvailable = false;
        private double cameraWeight = 0.3; // How much to trust camera vs odometry
        private long lastCameraUpdateTime = 0;
        private static final long CAMERA_TIMEOUT_MS = 1000; // Consider camera stale after 1 second
        
        /**
         * Constructor initializes the localization system
         * @param startX Starting X position in inches
         * @param startY Starting Y position in inches  
         * @param startHeading Starting heading in degrees
         */
        public PositionLocalization(double startX, double startY, double startHeading) {
            double clampedX = Math.max(0, Math.min(FIELD_SIZE_INCHES, startX));
            double clampedY = Math.max(0, Math.min(FIELD_SIZE_INCHES, startY));
            double normalizedHeading = normalizeAngle(startHeading);
            
            // Initialize fused and camera positions to starting point
            this.fusedX = this.cameraX = clampedX;
            this.fusedY = this.cameraY = clampedY;
            this.fusedHeading = this.cameraHeading = normalizedHeading;
            
            // Odometry starts uninitialized
            this.odometryX = 72.0;
            this.odometryY = 72.0;
            this.odometryHeading = 0.0;
            this.odometryInitialized = false;
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
            // Check for odometry initialization from camera
            if (!odometryInitialized && cameraResult != null && cameraResult.isValid() && 
                cameraResult.getTa() > CommonDefs.LIMELIGHT_HEADING_TARGETAREA_THRESHOLD) {
                initializeOdometryFromCamera(cameraResult, handler);
            }
            
            // Update odometry-only position (if initialized)
            if (odometryInitialized) {
                updateOdometry(forwardTicks, straferTicks);
            }
            
            // Update camera-only position and fused position
            if (cameraResult != null && cameraResult.isValid()) {
                updateCameraPosition(cameraResult, handler, currentTimeMs);
                fuseCameraData(cameraResult, currentTimeMs, handler);
            } else {
                // Check if camera data is stale
                if (currentTimeMs - lastCameraUpdateTime > CAMERA_TIMEOUT_MS) {
                    cameraAvailable = false;
                    // When camera is lost, use odometry to update camera position
                    if (odometryInitialized) {
                        propagateCameraPositionFromOdometry(forwardTicks, straferTicks);
                    }
                }
            }
            
            // Ensure all positions stay within field bounds
            constrainAllPositionsToField();
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
            
            // Convert to field coordinates considering robot orientation (use odometry heading)
            double headingRad = Math.toRadians(odometryHeading);
            double cosHeading = Math.cos(headingRad);
            double sinHeading = Math.sin(headingRad);
            
            // Transform pod movements to field coordinate system
            double deltaX = deltaForwardInches * cosHeading - deltaStraferInches * sinHeading;
            double deltaY = deltaForwardInches * sinHeading + deltaStraferInches * cosHeading;
            
            // Update odometry-only position
            odometryX += deltaX;
            odometryY += deltaY;
            
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
                // Convert camera coordinates to field coordinates using handler transform
                Pose camPose = handler.transformBotposeToFieldInches(botPose);
                double camX = camPose.getX();
                double camY = camPose.getY();
                double camHeading = normalizeAngle(Math.toDegrees(camPose.getHeading()));
                
                // Weighted fusion of odometry and camera data for fused position
                if (cameraAvailable) {
                    fusedX = odometryX * (1.0 - cameraWeight) + camX * cameraWeight;
                    fusedY = odometryY * (1.0 - cameraWeight) + camY * cameraWeight;
                    fusedHeading = blendAngles(odometryHeading, camHeading, cameraWeight);
                } else {
                    // First camera reading - use it more heavily to correct drift
                    fusedX = odometryX * 0.5 + camX * 0.5;
                    fusedY = odometryY * 0.5 + camY * 0.5;
                    fusedHeading = blendAngles(odometryHeading, camHeading, 0.5);
                }
                
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
         * Update camera-only position when camera is visible
         */
        private void updateCameraPosition(LLResult cameraResult, LimeLightHandler handler, long currentTimeMs) {
            Pose3D botPose = extractBotPose(cameraResult, handler);
            if (botPose != null) {
                Pose camPose = handler.transformBotposeToFieldInches(botPose);
                cameraX = camPose.getX();
                cameraY = camPose.getY();
                cameraHeading = normalizeAngle(Math.toDegrees(camPose.getHeading()));
            }
        }
        
        /**
         * When camera is lost, use odometry deltas to update camera position
         */
        private void propagateCameraPositionFromOdometry(long forwardTicks, long straferTicks) {
            // Calculate movement deltas
            long deltaForwardTicks = forwardTicks - previousForwardTicks;
            long deltaStraferTicks = straferTicks - previousStraferTicks;
            
            double deltaForwardInches = deltaForwardTicks * INCHES_PER_TICK;
            double deltaStraferInches = deltaStraferTicks * INCHES_PER_TICK;
            
            // Convert to field coordinates using camera heading
            double headingRad = Math.toRadians(cameraHeading);
            double cosHeading = Math.cos(headingRad);
            double sinHeading = Math.sin(headingRad);
            
            // Transform pod movements to field coordinate system
            double deltaX = deltaForwardInches * cosHeading - deltaStraferInches * sinHeading;
            double deltaY = deltaForwardInches * sinHeading + deltaStraferInches * cosHeading;
            
            // Update camera position with odometry deltas
            cameraX += deltaX;
            cameraY += deltaY;
        }
        
        /**
         * Ensure all positions stay within field boundaries
         */
        private void constrainAllPositionsToField() {
            fusedX = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, fusedX));
            fusedY = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, fusedY));
            
            cameraX = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, cameraX));
            cameraY = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, cameraY));
            
            odometryX = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, odometryX));
            odometryY = Math.max(0.0, Math.min(FIELD_SIZE_INCHES, odometryY));
        }
        
        /**
         * Get current fused position (default - combination of camera and odometry)
         * @return Pose object with fused position and heading
         */
        public Pose getCurrentPosition() {
            return getCurrentFusedPosition();
        }
        
        /**
         * Get current fused position
         * @return Pose object with fused position and heading
         */
        public Pose getCurrentFusedPosition() {
            return new Pose(fusedX, fusedY, Math.toRadians(fusedHeading));
        }
        
        /**
         * Get current camera-only position
         * @return Pose object with camera position and heading
         */
        public Pose getCurrentCameraPosition() {
            return new Pose(cameraX, cameraY, Math.toRadians(cameraHeading));
        }
        
        /**
         * Get current odometry-only position
         * @return Pose object with odometry position and heading
         */
        public Pose getCurrentOdometryPosition() {
            return new Pose(odometryX, odometryY, Math.toRadians(odometryHeading));
        }
        
        /**
         * Get current fused X position in inches
         */
        public double getCurrentX() {
            return fusedX;
        }
        
        /**
         * Get current fused Y position in inches  
         */
        public double getCurrentY() {
            return fusedY;
        }
        
        /**
         * Get current fused heading in degrees
         */
        public double getCurrentHeading() {
            return fusedHeading;
        }
        
        /**
         * Get distance to red basket (144, 144) using fused position
         */
        public double getDistanceToRedBasket() {
            return Math.sqrt(Math.pow(144.0 - fusedX, 2) + Math.pow(144.0 - fusedY, 2));
        }
        
        /**
         * Get distance to blue basket (0, 144) using fused position
         */
        public double getDistanceToBlueBasket() {
            return Math.sqrt(Math.pow(fusedX, 2) + Math.pow(144.0 - fusedY, 2));
        }
        
        /**
         * Get angle to red basket in degrees using fused position
         */
        public double getAngleToRedBasket() {
            return Math.toDegrees(Math.atan2(144.0 - fusedY, 144.0 - fusedX));
        }
        
        /**
         * Get angle to blue basket in degrees using fused position
         */
        public double getAngleToBlueBasket() {
            return Math.toDegrees(Math.atan2(144.0 - fusedY, 0.0 - fusedX));
        }
        
        /**
         * Check if camera localization is currently available and recent
         */
        public boolean isCameraAvailable() {
            return cameraAvailable;
        }
        
        /**
         * Reset all positions to specified coordinates
         */
        public void resetPosition(double x, double y, double heading) {
            double clampedX = Math.max(0, Math.min(FIELD_SIZE_INCHES, x));
            double clampedY = Math.max(0, Math.min(FIELD_SIZE_INCHES, y));
            double normalizedHeading = normalizeAngle(heading);
            
            // Reset fused and camera positions
            fusedX = cameraX = clampedX;
            fusedY = cameraY = clampedY;
            fusedHeading = cameraHeading = normalizedHeading;
            
            // Reset odometry to uninitialized state
            odometryX = 72.0;
            odometryY = 72.0;
            odometryHeading = 0.0;
            odometryInitialized = false;
            
            cameraAvailable = false;
        }
        
        /**
         * Set camera fusion weight (0.0 = pure odometry, 1.0 = pure camera)
         */
        public void setCameraWeight(double weight) {
            this.cameraWeight = Math.max(0.0, Math.min(1.0, weight));
        }
        
        /**
         * Initialize odometry position from first valid camera reading
         */
        private void initializeOdometryFromCamera(LLResult cameraResult, LimeLightHandler handler) {
            Pose3D botPose = extractBotPose(cameraResult, handler);
            if (botPose != null) {
                odometryX = CommonDefs.ConvertCameraPosToInches_x(botPose.getPosition().x);
                odometryY = CommonDefs.ConvertCameraPosToInches_y(botPose.getPosition().y);
                odometryHeading = normalizeAngle(botPose.getOrientation().getYaw(AngleUnit.DEGREES));
                odometryInitialized = true;
            }
        }
        
        /**
         * Check if odometry has been initialized
         */
        public boolean isOdometryInitialized() {
            return odometryInitialized;
        }
    }
}