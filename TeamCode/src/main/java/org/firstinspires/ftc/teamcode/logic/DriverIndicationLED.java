package org.firstinspires.ftc.teamcode.logic;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Handles control of a REV Robotics Blinkin LED Driver (3118-0808-0002).
 * This device is controlled via a PWM signal, like a servo. It displays
 * preset patterns and colors based on the received PWM value.
 */
public class DriverIndicationLED {

    private final Servo blinkin;

    // These values correspond to solid colors on the REV Blinkin LED Driver.
    // The values are based on the standard servo PWM range (0.0 to 1.0).
    public static final double PATTERN_RED = 0.3;
    public static final double PATTERN_GREEN = 0.5;
    public static final double PATTERN_BLUE = 0.6;
    public static final double PATTERN_GRAY = 0.9;
    
    // Blinking patterns
    public static final double PATTERN_BLINK_RED = 0.15;
    public static final double PATTERN_BLINK_GREEN = 0.35;
    public static final double PATTERN_BLINK_BLUE = 0.25;
    
    // Blinking state
    private boolean blinkingMode = false;
    private double currentSolidPattern = PATTERN_GRAY;
    private double currentBlinkPattern = PATTERN_GRAY;

    /**
     * Initializes the Blinkin LED driver.
     * @param hardwareMap The robot's hardware map.
     */
    public DriverIndicationLED(HardwareMap hardwareMap) {
        // The Blinkin is configured as a servo in the robot configuration.
        this.blinkin = hardwareMap.get(Servo.class, "lednotification");
    }

    /**
     * Sets the LED to solid red.
     */
    public void setRed() {
        currentSolidPattern = PATTERN_RED;
        currentBlinkPattern = PATTERN_BLINK_RED;
        updateLED();
    }

    /**
     * Sets the LED to solid green.
     */
    public void setGreen() {
        currentSolidPattern = PATTERN_GREEN;
        currentBlinkPattern = PATTERN_BLINK_GREEN;
        updateLED();
    }

    /**
     * Sets the LED to solid blue.
     */
    public void setBlue() {
        currentSolidPattern = PATTERN_BLUE;
        currentBlinkPattern = PATTERN_BLINK_BLUE;
        updateLED();
    }

    /**
     * Sets the LED to a neutral/off state.
     */
    public void off() {
        currentSolidPattern = PATTERN_GRAY;
        currentBlinkPattern = PATTERN_GRAY;
        blinkingMode = false;
        blinkin.setPosition(PATTERN_GRAY);
    }

    /**
     * Enable or disable blinking mode
     * @param blink true to enable blinking, false for solid color
     */
    public void setBlinking(boolean blink) {
        blinkingMode = blink;
        updateLED();
    }
    
    /**
     * Get current blinking state
     * @return true if blinking is enabled
     */
    public boolean isBlinking() {
        return blinkingMode;
    }
    
    /**
     * Update the LED pattern based on current color and blinking mode
     */
    private void updateLED() {
        if (blinkingMode) {
            blinkin.setPosition(currentBlinkPattern);
        } else {
            blinkin.setPosition(currentSolidPattern);
        }
    }

    /**
     * Allows setting a custom pattern by providing the raw PWM value.
     * @param pwmValue The PWM value (0.0 to 1.0) for the desired pattern.
     */
    public void setPattern(double pwmValue) {
        if (pwmValue >= 0.0 && pwmValue <= 1.0) {
            blinkingMode = false;
            currentSolidPattern = pwmValue;
            blinkin.setPosition(pwmValue);
        }
    }
}
