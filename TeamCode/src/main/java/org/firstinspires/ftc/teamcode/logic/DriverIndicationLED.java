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
        blinkin.setPosition(PATTERN_RED);
    }

    /**
     * Sets the LED to solid green.
     */
    public void setGreen() {
        blinkin.setPosition(PATTERN_GREEN);
    }

    /**
     * Sets the LED to solid blue.
     */
    public void setBlue() {
        blinkin.setPosition(PATTERN_BLUE);
    }

    /**
     * Sets the LED to a neutral/off state.
     */
    public void off() {
        blinkin.setPosition(PATTERN_GRAY);
    }

    /**
     * Allows setting a custom pattern by providing the raw PWM value.
     * @param pwmValue The PWM value (0.0 to 1.0) for the desired pattern.
     */
    public void setPattern(double pwmValue) {
        if (pwmValue >= 0.0 && pwmValue <= 1.0) {
            blinkin.setPosition(pwmValue);
        }
    }
}
