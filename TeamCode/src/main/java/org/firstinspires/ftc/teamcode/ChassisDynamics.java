package org.firstinspires.ftc.teamcode;

/**
 * Converts a desired chassis motion into per-wheel VOLTAGES.
 *
 * WHY THIS IS FEEDFORWARD AND NOT A NEW PLANT MODEL
 * The obvious design is to have the LQR output forces or accelerations and
 * invert the robot's mass and inertia to get voltages. That is not what this
 * does, on purpose. The follower's LQR is solved for a velocity-level model
 * (control = velocity), because a drivetrain closed around wheel velocity
 * behaves like a velocity source. Switching the LQR to output accelerations
 * would reinsert an integration the model does not know about, which shows up
 * as overshoot you cannot tune away. So the LQR keeps producing velocities,
 * and this class turns those velocities into the voltage that produces them:
 *
 *     V = Ks * sign(v)  +  Kv * v  +  Ka * a
 *
 * Ks  static friction, the voltage needed before the wheel moves at all
 * Kv  volts per inch/second of steady-state wheel speed
 * Ka  volts per inch/second^2 of wheel acceleration
 *
 * Ka EMPIRICALLY CAPTURES MASS. A heavier robot needs more volts to accelerate,
 * and that shows up directly in a measured Ka. That is why massKg and
 * momentOfInertia below are NOT in the main control path: measuring a robot's
 * moment of inertia accurately is genuinely hard, whereas Ka falls out of a
 * simple drive test. They are kept only for the feasibility check, which is
 * advisory.
 *
 * HOW TO MEASURE (about 15 minutes with a field tile and telemetry):
 *   Ks  ramp voltage up slowly until the wheels just break free. That voltage.
 *   Kv  drive at a few constant powers, record steady-state wheel speed,
 *       fit volts against speed. The slope is Kv.
 *   Ka  from the same runs, the gap between commanded voltage and Ks + Kv*v
 *       during the acceleration phase, divided by measured acceleration.
 */
public class ChassisDynamics {

    private final double ks, kv, ka;
    private final double nominalVoltage;
    private final double trackRadius;   // (halfTrackWidth + halfWheelBase), inches
    private final double maxVoltage;

    /**
     * Cap on how far voltage compensation may scale a command UP as the battery
     * sags. Dividing by the live battery voltage is the mathematically correct
     * inversion, but it is also exactly the wrong thing to do during a brownout:
     * the pack sags, the code demands more current to hold the same voltage, and
     * the sag deepens. This cap keeps the correct behaviour in the normal range
     * while refusing to chase a collapsing battery into a reset.
     */
    private static final double MAX_COMPENSATION_BOOST = 1.25;

    /** Advisory only, not used by the control path. See class comment. */
    public final double massKg, momentOfInertia;

    public ChassisDynamics(double ks, double kv, double ka,
                           double trackWidthInches, double wheelBaseInches,
                           double nominalVoltage, double maxVoltage,
                           double massKg, double momentOfInertia) {
        if (kv <= 0) throw new IllegalArgumentException("kv must be > 0; measure it, do not guess");
        if (ka < 0)  throw new IllegalArgumentException("ka must be >= 0");
        if (ks < 0)  throw new IllegalArgumentException("ks must be >= 0");
        if (trackWidthInches <= 0 || wheelBaseInches <= 0) {
            throw new IllegalArgumentException("track width and wheel base must be > 0");
        }
        this.ks = ks; this.kv = kv; this.ka = ka;
        this.trackRadius = (trackWidthInches + wheelBaseInches) / 2.0;
        this.nominalVoltage = nominalVoltage;
        this.maxVoltage = maxVoltage;
        this.massKg = massKg;
        this.momentOfInertia = momentOfInertia;
    }

    /**
     * Starting point for an 18in FTC mecanum on goBILDA 312rpm motors with the
     * standard 96mm wheel. This replaces an earlier version of this method that
     * shipped with a Kv of 0.0155, which was wrong: it implied a maximum
     * velocity of (12 - 0.08) / 0.0155 = 769 in/s. No FTC robot moves that fast;
     * that Kv was off by roughly 12x. Corrected below by deriving Kv from the
     * actual motor spec instead of guessing it.
     *
     * THE MATH, so you can redo it if your wheel size or motor differs:
     *   wheel circumference = pi * wheel diameter
     *                       = pi * (96mm / 25.4)          = 11.87 in
     *   free speed (no load) = (RPM / 60) * circumference
     *                        = (312 / 60) * 11.87          = 61.7 in/s
     *   Kv = (nominal voltage - Ks) / free speed
     *      = (12 - 0.10) / 61.7                            = 0.193 V per in/s
     *
     * Ks and Ka are NOT derivable from RPM. Ks is static friction and Ka
     * captures your robot's actual mass; both need a real bench test (see the
     * class comment above). The Ks and Ka here are reasonable starting guesses
     * for an 18in robot, nothing more.
     *
     * If your wheel diameter is not 96mm, recompute free speed with your real
     * diameter before trusting this Kv.
     */
    public static ChassisDynamics estimatedDefaults() {
        return new ChassisDynamics(0.10, 0.193, 0.0022, 14.0, 14.0, 12.0, 12.0, 14.0, 0.45);
    }

    /**
     * Robot-frame chassis velocity and acceleration to wheel linear velocities.
     * Sign convention MUST match the mixer in LQRPathFollower.toWheelPowers. Right handed:
     * vy points LEFT and omega is counterclockwise.
     *   FL = vx - vy - r*w    FR = vx + vy + r*w
     *   BL = vx + vy - r*w    BR = vx - vy + r*w
     * (The old version had vy and omega mirrored, which is the stick convention, not the one the path math uses.)
     */
    public double[] wheelVelocities(double vx, double vy, double omega) {
        double r = trackRadius;
        return new double[]{
                vx - vy - r * omega,
                vx + vy + r * omega,
                vx + vy - r * omega,
                vx - vy + r * omega
        };
    }

    /**
     * Full conversion: chassis motion to motor powers in [-1, 1].
     *
     * @param batteryVoltage live reading. Pass nominalVoltage to disable
     *                       compensation entirely.
     * @return {FL, FR, BL, BR} powers
     */
    public double[] toMotorPowers(double vx, double vy, double omega,
                                  double ax, double ay, double alpha,
                                  double batteryVoltage) {
        double[] v = wheelVelocities(vx, vy, omega);
        double[] a = wheelVelocities(ax, ay, alpha);

        double[] volts = new double[4];
        for (int i = 0; i < 4; i++) {
            volts[i] = ks * Math.signum(v[i]) + kv * v[i] + ka * a[i];
            // Ks must not inject a kick when the wheel is meant to be still.
            if (Math.abs(v[i]) < 1e-6 && Math.abs(a[i]) < 1e-6) volts[i] = 0;
        }

        // Scale down together if any wheel exceeds the supply, so the DIRECTION
        // of travel is preserved. Clamping wheels independently would change
        // the commanded heading, which is a subtle and very annoying bug.
        double peak = 0;
        for (double x : volts) peak = Math.max(peak, Math.abs(x));
        if (peak > maxVoltage) {
            double k = maxVoltage / peak;
            for (int i = 0; i < 4; i++) volts[i] *= k;
        }

        double effective = batteryVoltage;
        if (effective < 1e-3) effective = nominalVoltage;   // bad reading, fail safe
        double compensation = nominalVoltage / effective;
        if (compensation > MAX_COMPENSATION_BOOST) compensation = MAX_COMPENSATION_BOOST;

        double[] powers = new double[4];
        for (int i = 0; i < 4; i++) {
            double p = (volts[i] / nominalVoltage) * compensation;
            powers[i] = Math.max(-1, Math.min(1, p));
        }
        return powers;
    }

    /** True when the wheel voltages for this motion fit inside the supply. */
    public boolean isFeasible(double vx, double vy, double omega,
                              double ax, double ay, double alpha) {
        double[] v = wheelVelocities(vx, vy, omega);
        double[] a = wheelVelocities(ax, ay, alpha);
        for (int i = 0; i < 4; i++) {
            double volts = ks * Math.signum(v[i]) + kv * v[i] + ka * a[i];
            if (Math.abs(volts) > maxVoltage) return false;
        }
        return true;
    }

    /** Highest sustainable straight-line speed, from the voltage budget. */
    public double maxAchievableVelocity() {
        return (maxVoltage - ks) / kv;
    }

    public double trackRadius() { return trackRadius; }
}