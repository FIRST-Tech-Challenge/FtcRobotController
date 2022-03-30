package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.carouselspinner;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

/**
 * A class to control the robot carousel spinner
 */
public class CarouselSpinner implements Controllable<Void> {
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

    private final double sleepPercentage = .90;

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

        Thread.sleep((long) (duckSleepTime * sleepPercentage)); //Sleep for the majority of the duck spin time

        //Spin wait for the rest
        while (t.milliseconds() < duckSleepTime) {
            Thread.yield();
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
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
        Thread.sleep((long) (duckSleepTime * sleepPercentage)); //Sleep for the majority of the duck spin time

        //Spin wait for the rest
        while (t.milliseconds() < duckSleepTime) {
            Thread.yield();
            if (Thread.currentThread().isInterrupted()) {
                throw new InterruptedException();
            }
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
    public void halt() {
        spinnerServo.setPower(0);
    }

    /**
     * Allows control of a carousel spinner
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     * @return Always returns null
     */
    @Override
    public Void gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        this.spinnerServo.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        return null;
    }
}
