package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction
 */
public class Encoder {
    private final static int CPS_STEP = 0x10000;

    private static double inverseOverflow(double input, double estimate) {
        // convert to uint16
        int real = (int) input & 0xffff;
        // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
        // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
        real += ((real % 20) / 4) * CPS_STEP;
        // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
        real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
        return real;
    }

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private DcMotorEx motor;
    private NanoClock clock;

    private Direction direction;

    private int lastPosition;
    private int velocityEstimateIdx;
    private double[] velocityEstimates;
    private double lastUpdateTime;

    public Encoder(DcMotorEx motor, NanoClock clock) {
        this.motor = motor;
        this.clock = clock;

        this.direction = Direction.FORWARD;

        this.lastPosition = 0;
        this.velocityEstimates = new double[3];
        this.lastUpdateTime = clock.seconds();
    }

    public Encoder(DcMotorEx motor) {
        this(motor, NanoClock.system());
    }

    public Direction getDirection() {
        return direction;
    }

    private int getMultiplier() {
        return getDirection().getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    /**
     * Gets the position from the underlying motor and adjusts for the set direction.
     * Additionally, this method updates the velocity estimates used for compensated velocity
     *
     * @return encoder position
     */
    public int getCurrentPosition() {
        int multiplier = getMultiplier();
        int currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimates[velocityEstimateIdx] = (currentPosition - lastPosition) / dt;
            velocityEstimateIdx = (velocityEstimateIdx + 1) % 3;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    /**
     * Gets the velocity directly from the underlying motor and compensates for the direction
     * See {@link #getCorrectedVelocity} for high (>2^15) counts per second velocities (such as on REV Through Bore)
     *
     * @return raw velocity
     */
    public double getRawVelocity() {
        int multiplier = getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    /**
     * Uses velocity estimates gathered in {@link #getCurrentPosition} to estimate the upper bits of velocity
     * that are lost in overflow due to velocity being transmitted as 16 bits.
     * CAVEAT: must regularly call {@link #getCurrentPosition} for the compensation to work correctly.
     *
     * @return corrected velocity
     */
    public double getCorrectedVelocity() {
        double median = velocityEstimates[0] > velocityEstimates[1]
                ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
        return inverseOverflow(getRawVelocity(), median);
    }
}
