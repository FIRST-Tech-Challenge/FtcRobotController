package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A class to control the robot carousel spinner
 */
public class CarouselSpinner {
    /**
     * How Fast the servo is to spin
     */
    private static final double servoPower = 1;
    /**
     * How long it takes to spin off a duck
     */
    private static final long duckSleepTime = 3100;
    /**
     * The continuous servo
     */
    protected final CRServo spinnerServo;

    /**
     * A constructor that sets up servo from Hardware map
     *
     * @param hardwareMap Hardware Map Object
     * @param deviceName  Name of continuous servo
     */
    public CarouselSpinner(HardwareMap hardwareMap, String deviceName) {
        spinnerServo = hardwareMap.crservo.get(deviceName);
    }

    /**
     * Spins off the red duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    public void spinOffRedDuck() throws InterruptedException {
        spinnerServo.setPower(-servoPower);
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < duckSleepTime) {
            Thread.sleep(20);
        }
        spinnerServo.setPower(0);

    }

    /**
     * Spins off the blue duck
     *
     * @throws InterruptedException raises exception if the OpMode is stopped
     */
    public void spinOffBlueDuck() throws InterruptedException {
        spinnerServo.setPower(servoPower);
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < duckSleepTime) {
            Thread.sleep(20);
        }
        spinnerServo.setPower(0);

    }

    /**
     * Spins in blue direction
     */
    public void setPowerBlueDirection() {
        spinnerServo.setPower(servoPower);
    }

    /**
     * Spins in red direction
     */
    public void setPowerRedDirection() {
        spinnerServo.setPower(-servoPower);
    }

    /**
     * Stops the spinner
     */
    public void stop() {
        spinnerServo.setPower(0);
    }
}
