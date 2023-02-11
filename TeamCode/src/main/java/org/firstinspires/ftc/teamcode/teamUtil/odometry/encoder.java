package org.firstinspires.ftc.teamcode.teamUtil.odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is just roadrunner stuff. some conversion from inches to mm changed, tbh I don't really understand it.
 */

public class encoder {
    private final static int CPS_STEP = 0x10000;

    private static double inverseOverflow(double input, double estimate) {
        // convert to uint16
        int real = (int) input & 0xffff;

        // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
        // because the velocity is always a multiple of 20cps due to Expansion Hub's 50 ms measurement window
        real += ((real % 20) / 4) * CPS_STEP;

        //estimate-based correction: it finds the nearest multiple of 5
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
            return  multiplier;
        }
    }

    private DcMotorEx motor;

    private Direction direction;

    private int lastPosition;
    private int velocityEstimateIdx;
    private double[] velocityEstimates;
    private double lastUpdateTime;

    public encoder(DcMotorEx motor) {
        this.motor = motor;

        this.direction = Direction.FORWARD;

        this.lastPosition = 0;
        this.velocityEstimates = new double[3];
        this.lastUpdateTime = System.nanoTime()/1e9;
    }

    public Direction getDirection() {
        return direction;
    }

    private int getMultiplier() {
        return getDirection().getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        int multiplier = getMultiplier();
        int currentPosition = motor.getCurrentPosition() * multiplier;
        if(currentPosition != lastPosition){
            double currentTime = System.nanoTime()/1E9;
            double dt = currentTime - lastUpdateTime;
            velocityEstimates[velocityEstimateIdx] = (currentPosition - lastPosition) / dt;
            velocityEstimateIdx = (velocityEstimateIdx + 1) % 3;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    /**
     * not to be used with the REV encoders
     * @return raw velocity
     */
    private double getRawVelocity() {
        int multiplier = getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    /**
     * MUST call {@link #getCurrentPosition()} regularly for this to work
     * @return corrected velocity
     */
    public double getCorrectedVelocity() {
        double median = velocityEstimates[0] > velocityEstimates[1]
                ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
        return inverseOverflow(getRawVelocity(), median);
    }
}
