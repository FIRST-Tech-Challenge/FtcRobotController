package org.firstinspires.ftc.teamcode;

/**
 * Thermal protection for transfer stage 1 (goBILDA 5203 1:1, about 9 A at stall). A stall is a low
 * encoder speed AND high motor current while stage 1 is commanded on; requiring both means an
 * unplugged encoder (0 RPM, normal current) never cuts the motor. A stall held for
 * {@link #STALL_DETECT_SECONDS} cuts stage 1 for {@link #COOLDOWN_SECONDS}, then retries. After
 * {@link #MAX_RETRIES} trips in a row without a clean run, stage 1 stays off (locked out) until the
 * command drops to zero or the caller resets the guard. {@link TransferSubsystem} never applies the
 * guard to shot feeds: both stages always get full power while feeding, and each feed resets it.
 *
 * <p>The thresholds below are starting values, not bench measurements: check the stage 1 RPM and
 * current telemetry during normal loading with a healthy motor and keep both well clear of them.
 */
public final class TransferStallGuard {
    /** Below 25% of the 6000 RPM free speed counts as stalled (if current is also high). */
    public static final double STALL_MAX_RPM = 1500.0;
    /**
     * Free spin is about 0.25 A and stall about 9.2 A at 12 V, roughly linear in speed, so a real stall
     * below 1500 RPM draws about 6.4-7 A even on a sagging battery. 4 A (the first value) tripped during
     * ordinary loading (2026-11-18): it is only about 3500 RPM of real speed, so it counted as a stall
     * whenever the encoder read low. At 6 A even a 0 RPM encoder reading needs the motor really near
     * stall (about 2100 RPM or slower) before the guard cuts it.
     */
    public static final double STALL_MIN_CURRENT_AMPS = 6.0;
    /** Longer than a 0.20 s timed feed window and motor spin-up, short enough to avoid cooking it. */
    public static final double STALL_DETECT_SECONDS = 0.75;
    public static final double COOLDOWN_SECONDS = 1.0;
    /** Continuous clean running that forgives earlier trips. */
    public static final double CLEAR_SECONDS = 1.0;
    public static final int MAX_RETRIES = 3;

    public enum State { OK, COOLING, LOCKED_OUT }

    private State state = State.OK;
    private double stalledSince = Double.NaN, cleanSince = Double.NaN;
    private double coolingUntil = Double.NEGATIVE_INFINITY;
    private int trips = 0;
    private int totalTrips = 0;

    /**
     * Returns the power stage 1 may actually use. {@code rpm} and {@code amps} are the latest
     * measurements (NaN when not yet sampled, which never counts as a stall).
     */
    public double filter(double now, double commandedPower, double rpm, double amps) {
        if (commandedPower <= 0) { reset(); return commandedPower; }
        if (state == State.LOCKED_OUT) return 0.0;
        if (state == State.COOLING) {
            if (now < coolingUntil) return 0.0;
            state = State.OK; // Retry; fresh samples must show a new stall before the next trip.
            stalledSince = cleanSince = Double.NaN;
        }
        boolean stalled = Math.abs(rpm) < STALL_MAX_RPM && amps >= STALL_MIN_CURRENT_AMPS;
        if (stalled) {
            cleanSince = Double.NaN;
            if (Double.isNaN(stalledSince)) stalledSince = now;
            if (now - stalledSince >= STALL_DETECT_SECONDS) {
                trips++; totalTrips++;
                stalledSince = Double.NaN;
                if (trips >= MAX_RETRIES) { state = State.LOCKED_OUT; }
                else { state = State.COOLING; coolingUntil = now + COOLDOWN_SECONDS; }
                return 0.0;
            }
        } else {
            stalledSince = Double.NaN;
            if (Double.isNaN(cleanSince)) cleanSince = now;
            if (now - cleanSince >= CLEAR_SECONDS) trips = 0;
        }
        return commandedPower;
    }

    /** Clears cooldown, lockout and the trip count (the lifetime count is kept for telemetry). */
    public void reset() {
        state = State.OK; trips = 0;
        stalledSince = cleanSince = Double.NaN;
        coolingUntil = Double.NEGATIVE_INFINITY;
    }

    public State state() { return state; }
    public boolean cuttingPower() { return state != State.OK; }
    public int consecutiveTrips() { return trips; }
    public int totalTrips() { return totalTrips; }
    public double stalledSeconds(double now) { return Double.isNaN(stalledSince) ? 0 : now - stalledSince; }
}
