package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem controlling the robot's gate mechanism.
 * Connects to Control Hub Servo Port 5 configured as "gateServo".
 */
public class Gate {

    public final static double DEFAULT_POSITION = 0.25; //.25
    public final static double OPEN_POSITION = 0.02; //.02

    public static double TEST_OPEN_POSTITION = 0.25;
    public static double TEST_DEFAULT_POSITION = 0.02;

    public Servo gateServo;
    private boolean isOpen = false;
    private double currentPosition = DEFAULT_POSITION;

    public Gate(HardwareMap hardwareMap) {
        if (hardwareMap != null) {
            try {
                gateServo = hardwareMap.get(Servo.class, "gateServo");
                close();
            } catch (Exception e) {
                gateServo = null;
            }
        }
    }

    /**
     * Opens the gate by moving the servo to OPEN_POSITION.
     */
    public void open() {
        setPosition(OPEN_POSITION);
        isOpen = true;
    }

    /**
     * Closes the gate / returns it to the default position (DEFAULT_POSITION).
     */
    public void close() {
        setPosition(DEFAULT_POSITION);
        isOpen = false;
    }

    public void testClose() {
        setPosition(TEST_DEFAULT_POSITION);
        isOpen = false;
    }

    public void testOpen() {
        setPosition(TEST_OPEN_POSTITION);
        isOpen = true;
    }

    /**
     * Sets the servo to a specific target position clamped between 0.0 and 1.0.
     *
     * @param position Target servo position between 0.0 and 1.0.
     */
    public void setPosition(double position) {
        double clamped = Math.max(0.0, Math.min(1.0, position));
        currentPosition = clamped;
        if (gateServo != null) {
            gateServo.setPosition(clamped);
        }
    }

    /**
     * Returns the last commanded or current position of the servo.
     */
    public double getPosition() {
        if (gateServo != null) {
            return gateServo.getPosition();
        }
        return currentPosition;
    }

    /**
     * Returns whether the gate is currently commanded open.
     */
    public boolean isOpen() {
        return isOpen;
    }
}
