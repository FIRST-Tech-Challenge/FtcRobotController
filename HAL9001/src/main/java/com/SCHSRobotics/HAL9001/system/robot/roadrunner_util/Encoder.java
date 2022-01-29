package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.jetbrains.annotations.NotNull;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing without changing the corresponding
 * slot's motor direction
 * <p>
 * Creation Date: 1/8/21
 *
 * @author Roadrunner Quickstart
 * @version 1.0.0
 * @since 1.1.1
 */
public class Encoder {
    //A constant used to fix the counts per second integer overflow error that can occur with high-res encoders.
    private final static int CPS_STEP = 0x10000;
    //The motor object used to read encoder positions.
    private final DcMotorEx motor;
    //A nanosecond clock to keep time.
    private final NanoClock clock;
    //The encoder's last velocity estimates.
    private final MovingStatistics velocityEstimates;
    //The direction of the encoder.
    private Direction direction;
    //The encoder's last position.
    private int lastPosition;
    //The encoder's last update time.
    private double lastUpdateTime;

    /**
     * The constructor for encoder.
     *
     * @param motor The motor object used to read encoder positions.
     * @param clock A nanosecond clock to keep time.
     */
    public Encoder(DcMotorEx motor, @NotNull NanoClock clock) {
        this.motor = motor;
        this.clock = clock;

        this.direction = Direction.FORWARD;

        this.lastPosition = 0;
        this.velocityEstimates = new MovingStatistics(5);
        this.lastUpdateTime = clock.seconds();
    }

    /**
     * The constructor for encoder.
     *
     * @param motor The motor object used to read encoder positions.
     */
    public Encoder(DcMotorEx motor) {
        this(motor, NanoClock.system());
    }

    /**
     * Fixes the encoder counts per second integer overflow error.
     *
     * @param input    The raw encoder value.
     * @param estimate The estimated encoder value.
     * @return The corrected encoder velocity.
     */
    private static double inverseOverflow(double input, double estimate) {
        double real = input;
        while (Math.abs(estimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(estimate - real) * CPS_STEP;
        }
        return real;
    }

    /**
     * Gets the encoder direction.
     *
     * @return The encoder direction.
     */
    public Direction getDirection() {
        return direction;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state.
     *
     * @param direction either reverse or forward depending on if encoder counts should be negated.
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Gets the encoder's current position.
     *
     * @return The encoder's current position.
     */
    public int getCurrentPosition() {
        int multiplier = direction.getMultiplier();
        int currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimates.add((currentPosition - lastPosition) / dt);
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    /**
     * Get the encoder's raw velocity.
     *
     * @return The encoder's raw velocity.
     */
    public double getRawVelocity() {
        int multiplier = direction.getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    /**
     * Get the encoder's velocity. Corrected for integer overflow.
     *
     * @return Get the encoder's velocity. Corrected for integer overflow.
     */
    public double getCorrectedVelocity() {
        return inverseOverflow(getRawVelocity(), velocityEstimates.getMean());
    }

    /**
     * The direction of the encoder.
     */
    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        //A multiplier to set the encoder direction.
        private final int multiplier;

        /**
         * The constructor for the Direction enum.
         *
         * @param multiplier The multiplier to apply when this direction is selected.
         */
        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        /**
         * Gets the multiplier associated with this direction.
         *
         * @return The multiplier associated with this direction.
         */
        public int getMultiplier() {
            return multiplier;
        }
    }
}
