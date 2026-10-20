package org.firstinspires.ftc.teamcode.pollen;

/** INIT-only start delay, bounded by the route budget and frozen on START. */
public final class PollenStartDelay {
    public static final double DELAY_INCREMENT = 0.5;
    private final double maximumDelay;
    private double delay;
    private boolean locked;
    private boolean previousUp, previousDown;

    public PollenStartDelay(double defaultDelay, double routeSeconds, double deadline) {
        if (!Double.isFinite(defaultDelay) || defaultDelay < 0
                || !Double.isFinite(routeSeconds) || routeSeconds <= 0
                || !Double.isFinite(deadline) || routeSeconds >= deadline - 1) {
            throw new IllegalArgumentException("Invalid start delay budget");
        }
        double limit = 0;
        // Same strict budget as PollenSequence; cap long waits at 20 seconds.
        while (limit + DELAY_INCREMENT <= 20
                && routeSeconds + limit + DELAY_INCREMENT < deadline - 1) {
            limit += DELAY_INCREMENT;
        }
        maximumDelay = limit;
        delay = Math.min(maximumDelay,
                Math.round(defaultDelay / DELAY_INCREMENT) * DELAY_INCREMENT);
    }

    public void update(boolean up, boolean down) {
        if (locked) return;
        if (up && !down && !previousUp) delay = Math.min(maximumDelay, delay + DELAY_INCREMENT);
        if (down && !up && !previousDown) delay = Math.max(0, delay - DELAY_INCREMENT);
        previousUp = up;
        previousDown = down;
    }

    public void lock() { locked = true; }
    public boolean locked() { return locked; }
    public double delaySeconds() { return delay; }
    public double maximumDelaySeconds() { return maximumDelay; }
}
