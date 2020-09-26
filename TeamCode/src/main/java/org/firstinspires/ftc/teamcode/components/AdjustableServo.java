package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Adjustable;
import org.firstinspires.ftc.teamcode.support.hardware.Configurable;

/**
 * Wrapper around hardware Servo that allows left / center / right stops to be adjusted
 *  and translates arbitrary degree range specified by <code>rangeMin</code>
 *  and <code>rangeMax</code> constructor arguments to appropriate servo positions
 */
public class AdjustableServo extends Logger<AdjustableServo> implements Configurable {
    private String deviceName;
    private Servo servo;
    private double position = 0.0; // position in degrees

    private boolean adjustmentMode = false;
    private double left = 0.2;   // left servo stop
    private double center = 0.5; // central servo position
    private double right = 0.8;  // right servo stop

    private double rangeMin;
    private double rangeMax;

    public AdjustableServo() {
        this(-90, 90);
    }

    public AdjustableServo(double rangeMin, double rangeMax) {
        if (rangeMin >= rangeMax) {
            throw new IllegalArgumentException("rangeMin must be less than rangeMax");
        }
        this.rangeMin = rangeMin;
        this.rangeMax = rangeMax;
    }

    public void configure(HardwareMap hardwareMap, String deviceName) {
        this.servo = hardwareMap.get(Servo.class, deviceName);
        this.servo.setDirection(Servo.Direction.REVERSE);
        this.deviceName = deviceName;
    }

    @Override
    public String getUniqueName() {
        return deviceName;
    }

    @Override
    public void setAdjustmentMode(boolean adjustmentMode) {
        this.adjustmentMode = adjustmentMode;
    }

    @Adjustable(min = 0.0f, max = 1.0f, step = 0.001f)
    public double getLeft() {
        return left;
    }
    public void setLeft(double left) {
        this.left = left;
        if (adjustmentMode) setPosition(rangeMin);
    }

    @Adjustable(min = 0.0f, max = 1.0f, step = 0.001f)
    public double getCenter() {
        return center;
    }
    public void setCenter(double center) {
        this.center = center;
        if (adjustmentMode) setPosition((rangeMin + rangeMax) / 2);
    }

    @Adjustable(min = 0.0f, max = 1.0f, step = 0.001f)
    public double getRight() {
        return right;
    }
    public void setRight(double right) {
        this.right = right;
        if (adjustmentMode) setPosition(rangeMax);
    }

    /**
     * Returns servo position in degrees in <code>rangeMin</code> to <code>rangeMax</code> range
     *  with <code>rangeMin</code> being the leftmost position and <code>rangeMax</code>
     *  being the rightmost position.
     * Note that position returned is the last position set on servo and may not match its actual position.
     */
    public double getPosition() {
        return position;
    }

    /**
     * Sets servo position in degrees in <code>rangeMin</code> to <code>rangeMax</code> range
     * with <code>rangeMin</code> being the leftmost position and <code>rangeMax</code>
     * being the rightmost position.
     * @param degrees new position to turn servo to
     */
    public void setPosition(double degrees) {
        if (degrees < rangeMin || degrees > rangeMax) {
            throw new IllegalArgumentException("Servo position must be between " + rangeMin
                    + " and " + rangeMax + " degrees");
        }
        double hwPosition; // actual hardware position set on servo
        double halfPoint = (rangeMin + rangeMax) / 2;
        double halfDistance = (rangeMax - rangeMin) / 2;
        if (degrees < halfPoint) {
            // transform "rangeMin / half point" to "left / center" interval
            hwPosition = (degrees - rangeMin) / halfDistance * (center - left) + left;
        } else {
            // transform "half point / rangeMax" to "center / right" interval
            hwPosition = (degrees - halfPoint) / halfDistance * (right - center) + center;
        }
        verbose("position: %.2f -> %.2f, hw: %.2f", position, degrees, hwPosition);
        servo.setPosition(hwPosition);
        position = degrees;
    }

    /**
     * Adjusts current servo position by specified number of degrees. This is a convenience method
     *  that reads current servo position and adjusts it by specified amount making sure the result
     *  does not fall outside of <code>rangeMin</code> to <code>rangeMax</code> degree range.
     * @param degrees number of degrees (positive or negative) to adjust current servo position by
     */
    public void adjustPosition(double degrees) {
        double newPosition = getPosition() + degrees;
        if (newPosition < rangeMin) newPosition = rangeMin;
        if (newPosition > rangeMax) newPosition = rangeMax;
        setPosition(newPosition);
    }

    /**
     * Resets servo position back to center
     */
    public void reset() {
        if (servo==null) return;
        position = (rangeMin + rangeMax) / 2;
        servo.setPosition(center);
    }
}
