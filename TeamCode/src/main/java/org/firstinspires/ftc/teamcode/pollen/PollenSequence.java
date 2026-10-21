package org.firstinspires.ftc.teamcode.pollen;

/** Hardware-independent, nonblocking autonomous sequence. Times are seconds. */
public final class PollenSequence {
    public static final class Step {
        public final String name;
        public final double seconds, forward, strafe, turn;
        public final boolean shoot;

        private Step(String name, double seconds, double forward, double strafe,
                     double turn, boolean shoot) {
            if (name == null || !Double.isFinite(seconds) || seconds <= 0
                    || !Double.isFinite(forward) || !Double.isFinite(strafe)
                    || !Double.isFinite(turn) || Math.abs(forward) > 1
                    || Math.abs(strafe) > 1 || Math.abs(turn) > 1) {
                throw new IllegalArgumentException("Invalid route step");
            }
            this.name = name;
            this.seconds = seconds;
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.shoot = shoot;
        }

        public static Step drive(String name, double seconds, double forward,
                                 double strafe, double turn) {
            return new Step(name, seconds, forward, strafe, turn, false);
        }

        /** Qualify speed once, then feed continuously for this wall-clock duration. */
        public static Step continuousShoot(double seconds) {
            return new Step("SHOOT", seconds, 0, 0, 0, true);
        }
    }

    public static final class Output {
        public String phase;
        public double forward, strafe, turn, launcherVelocity, feederPower, activeFeedSeconds;
        public boolean finished;
        private Output(String phase) { this.phase = phase; }
    }

    private final Step[] steps;
    private final double targetVelocity, minVelocity, spinupTimeout, stableTime, deadline, startDelay;
    private int index;
    private double startedAt, stepStartedAt, stableSince = Double.NaN;
    private double feedingSince = Double.NaN, previousTime;
    private double activeFeedSeconds;
    private boolean wasFeeding;
    private boolean started, finished;
    private String result = "READY";
    private String warning = "";

    /** Drive-only route: callers do not need irrelevant launcher settings. */
    public static PollenSequence park(Step[] steps, double deadline, double startDelay) {
        if (steps != null) for (Step step : steps) {
            if (step != null && step.shoot) throw new IllegalArgumentException("Park route cannot shoot");
        }
        // Valid placeholders are unused because this route contains no shooting.
        return new PollenSequence(steps, 1, 1, 1, .25, deadline, startDelay);
    }

    public PollenSequence(Step[] steps, double targetVelocity, double minVelocity,
                          double spinupTimeout, double stableTime, double deadline) {
        this(steps, targetVelocity, minVelocity, spinupTimeout, stableTime, deadline, 0);
    }

    public PollenSequence(Step[] steps, double targetVelocity, double minVelocity,
                          double spinupTimeout, double stableTime, double deadline,
                          double startDelay) {
        if (steps == null || steps.length == 0 || !Double.isFinite(targetVelocity)
                || !Double.isFinite(minVelocity) || !Double.isFinite(spinupTimeout)
                || !Double.isFinite(stableTime) || !Double.isFinite(deadline)
                || minVelocity <= 0 || targetVelocity < minVelocity || stableTime <= 0
                || spinupTimeout <= stableTime || deadline <= 0
                || !Double.isFinite(startDelay) || startDelay < 0) {
            throw new IllegalArgumentException("Invalid sequence settings");
        }
        double worstCase = startDelay + worstCaseDuration(steps, spinupTimeout);
        // Leave room for transition loops; the independent deadline always stops outputs.
        if (worstCase >= deadline - 1) {
            throw new IllegalArgumentException("Route exceeds autonomous time budget");
        }
        this.steps = steps.clone();
        this.targetVelocity = targetVelocity;
        this.minVelocity = minVelocity;
        this.spinupTimeout = spinupTimeout;
        this.stableTime = stableTime;
        this.deadline = deadline;
        this.startDelay = startDelay;
    }

    /** Route wall-clock allowance, excluding start delay and loop-transition reserve. */
    public static double worstCaseDuration(Step[] steps, double spinupTimeout) {
        if (steps == null || steps.length == 0 || !Double.isFinite(spinupTimeout) || spinupTimeout <= 0) {
            throw new IllegalArgumentException("Invalid route budget");
        }
        double seconds = 0;
        for (Step step : steps) {
            if (step == null) throw new IllegalArgumentException("Null step");
            seconds += step.shoot ? spinupTimeout + step.seconds : step.seconds;
        }
        return seconds;
    }

    public void start(double now) {
        if (!Double.isFinite(now)) throw new IllegalArgumentException("Invalid time");
        index = 0;
        startedAt = previousTime = now;
        stepStartedAt = now + startDelay;
        activeFeedSeconds = 0;
        wasFeeding = false;
        stableSince = feedingSince = Double.NaN;
        finished = false;
        started = true;
        warning = "";
        result = "RUNNING";
    }

    public Output tick(double now, double measuredVelocity, boolean stopRequested) {
        if (stopRequested) return stop("STOPPED");
        if (finished) return terminal();
        if (!started) return new Output("READY - waiting for START");
        if (!Double.isFinite(now) || now < previousTime) return stop("INVALID CLOCK");
        double elapsedSinceTick = now - previousTime;
        previousTime = now;
        if (now - startedAt >= deadline) return stop("TIME LIMIT - STOPPED");
        if (now - startedAt < startDelay) return new Output("START DELAY");
        Step step = steps[index];
        Output out = new Output(step.name);
        if (!step.shoot) {
            if (now - stepStartedAt >= step.seconds) return advance(now);
            out.forward = step.forward;
            out.strafe = step.strafe;
            out.turn = step.turn;
            return out;
        }

        out.phase = "SPINUP";
        out.launcherVelocity = targetVelocity;
        if (!Double.isFinite(measuredVelocity)) {
            return stop("INVALID LAUNCHER VELOCITY - STOPPED");
        }
        // Count the time for which the preceding loop actually requested feeding.
        // This measures commands, not balls or verified physical feeder motion.
        if (wasFeeding) activeFeedSeconds += elapsedSinceTick;
        wasFeeding = false;
        out.activeFeedSeconds = activeFeedSeconds;
        boolean ready = Double.isFinite(measuredVelocity) && measuredVelocity >= minVelocity;
        if (!ready) stableSince = Double.NaN;
        else if (Double.isNaN(stableSince)) stableSince = now;
        boolean stable = ready && now - stableSince >= stableTime;

        if (Double.isNaN(feedingSince)) {
            if (now - stepStartedAt >= spinupTimeout) {
                warning = "Launcher not ready: shot skipped; continuing route";
                return advance(now);
            }
            if (stable) feedingSince = now;
        }
        if (!Double.isNaN(feedingSince)) {
            if (now - feedingSince >= step.seconds) return advance(now);
            out.phase = ready ? "CONTINUOUS FEED" : "CONTINUOUS FEED - speed below target band";
            out.feederPower = 1;
            wasFeeding = true;
        }
        return out;
    }

    private Output advance(double now) {
        index++;
        stepStartedAt = now;
        stableSince = feedingSince = Double.NaN;
        activeFeedSeconds = 0;
        wasFeeding = false;
        if (index == steps.length) return stop("DONE - all outputs zero");
        return new Output("NEXT: " + steps[index].name); // Zero outputs between steps.
    }

    public Output stop(String reason) {
        finished = true;
        result = reason;
        return terminal();
    }

    private Output terminal() {
        Output out = new Output(result);
        out.finished = true;
        return out;
    }

    public String warning() { return warning; }

    /** Output order: frontLeft, frontRight, backLeft, backRight. */
    public static double[] wheelPowers(Output out) {
        double[] powers = {out.forward + out.strafe + out.turn,
                out.forward - out.strafe - out.turn,
                out.forward - out.strafe + out.turn,
                out.forward + out.strafe - out.turn};
        double max = 1;
        for (double power : powers) max = Math.max(max, Math.abs(power));
        for (int i = 0; i < powers.length; i++) powers[i] /= max;
        return powers;
    }
}
