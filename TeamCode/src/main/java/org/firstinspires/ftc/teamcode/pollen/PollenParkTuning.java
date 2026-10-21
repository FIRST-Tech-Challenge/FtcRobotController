package org.firstinspires.ftc.teamcode.pollen;

/** Controller adjustments during INIT only. Pure Java, with no hardware commands. */
public final class PollenParkTuning {
    public static final double MIN_POWER = 0.05, MAX_POWER = 0.50;
    public static final double MIN_SECONDS = 0.05, MAX_SECONDS = 5.0;
    public static final double POWER_INCREMENT = 0.05, TIME_INCREMENT = 0.05;
    public static final double DELAY_INCREMENT = 0.5;

    /** Only power and drive time carry over between OpMode instances. */
    public static final class Snapshot {
        public final double power, seconds;
        private Snapshot(double power, double seconds) {
            this.power = power;
            this.seconds = seconds;
        }
    }

    private final double defaultPower, defaultSeconds, defaultDelay, deadline, holdSeconds;
    private double power, seconds, delay;
    private boolean locked, previousUp, previousDown, previousLeft, previousRight;
    private boolean previousLessPower, previousMorePower, previousReset;

    public PollenParkTuning(double defaultPower, double defaultSeconds, double defaultDelay,
            double deadline, double holdSeconds, Snapshot remembered) {
        if (!Double.isFinite(defaultPower) || defaultPower < MIN_POWER || defaultPower > MAX_POWER
                || !Double.isFinite(defaultSeconds) || defaultSeconds < MIN_SECONDS || defaultSeconds > MAX_SECONDS
                || !Double.isFinite(defaultDelay) || defaultDelay < 0
                || !Double.isFinite(holdSeconds) || holdSeconds <= 0
                || !Double.isFinite(deadline) || MAX_SECONDS + holdSeconds >= deadline - 1) {
            throw new IllegalArgumentException("Invalid parking tuning defaults or time budget");
        }
        this.defaultPower = defaultPower;
        this.defaultSeconds = defaultSeconds;
        this.defaultDelay = defaultDelay;
        this.deadline = deadline;
        this.holdSeconds = holdSeconds;
        power = remembered == null ? defaultPower : remembered.power;
        seconds = remembered == null ? defaultSeconds : remembered.seconds;
        delay = defaultDelay;
        clampDelay();
    }

    public void update(boolean up, boolean down, boolean left, boolean right,
            boolean lessPower, boolean morePower, boolean reset) {
        if (locked) return;
        if (reset) {
            // Reset dominates other keys, and a held reset never repeats.
            if (!previousReset) {
                power = defaultPower;
                seconds = defaultSeconds;
                delay = defaultDelay;
            }
        } else {
            if (right && !left && !previousRight) seconds = step(seconds, TIME_INCREMENT, MIN_SECONDS, MAX_SECONDS);
            if (left && !right && !previousLeft) seconds = step(seconds, -TIME_INCREMENT, MIN_SECONDS, MAX_SECONDS);
            if (morePower && !lessPower && !previousMorePower) power = step(power, POWER_INCREMENT, MIN_POWER, MAX_POWER);
            if (lessPower && !morePower && !previousLessPower) power = step(power, -POWER_INCREMENT, MIN_POWER, MAX_POWER);
            if (up && !down && !previousUp) delay += DELAY_INCREMENT;
            if (down && !up && !previousDown) delay -= DELAY_INCREMENT;
        }
        clampDelay();
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
        previousLessPower = lessPower;
        previousMorePower = morePower;
        previousReset = reset;
    }

    private static double step(double value, double amount, double min, double max) {
        return Math.max(min, Math.min(max, Math.round((value + amount) * 100.0) / 100.0));
    }

    private void clampDelay() {
        delay = Math.max(0, Math.min(maximumDelaySeconds(), Math.round(delay * 2.0) / 2.0));
    }

    public double maximumDelaySeconds() {
        double max = 0;
        while (max + DELAY_INCREMENT <= 20
                && seconds + holdSeconds + max + DELAY_INCREMENT < deadline - 1) {
            max += DELAY_INCREMENT;
        }
        return max;
    }

    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public double power() { return power; }
    public double seconds() { return seconds; }
    public double delaySeconds() { return delay; }
    public Snapshot snapshot() { return new Snapshot(power, seconds); }
}
