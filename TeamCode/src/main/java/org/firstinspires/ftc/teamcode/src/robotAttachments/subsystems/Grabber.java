package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * this is the class for our robot's intake subsystem
 */
@Deprecated
public class Grabber {
    /**
     * The position the servo must go to to open
     */
    private static final float openPosition = 0.5f;
    /**
     * The position the servo must go to to close
     */
    private static final float closePosition = .8f;
    /**
     * The servo that controls the intake
     */
    private final Servo grabberServo;
    /**
     * A boolean that says if the servo is open or closed
     */
    private boolean isOpen;

    /**
     * Constructs and initializes servo
     *
     * @param hardwareMap Hardware map object from OpMode
     * @param servoName   Servo Name
     */
    public Grabber(HardwareMap hardwareMap, String servoName) {
        grabberServo = hardwareMap.servo.get(servoName);
    }

    /**
     * Opens the grabber
     */
    public void open() {
        grabberServo.setPosition(openPosition);
        isOpen = true;
    }

    /**
     * Closes the grabber
     */
    public void close() {
        grabberServo.setPosition(closePosition);
        isOpen = false;
    }

    /**
     * Returns true if the grabber is open
     *
     * @return Returns true if the grabber is open, Returns false if the grabber is closed
     */
    public boolean isOpen() {
        return isOpen;
    }

    /**
     * Returns true if the grabber is closed
     *
     * @return Returns true if the grabber is closed, Returns false if the grabber is open
     */
    public boolean isClosed() {
        return !isOpen;
    }
}
