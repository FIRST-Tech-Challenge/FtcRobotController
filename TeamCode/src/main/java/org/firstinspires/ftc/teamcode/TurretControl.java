package org.firstinspires.ftc.teamcode;

/** Positional servo commands with minimal smoothing; external encoder correction is opt-in. */
public final class TurretControl {
    public static final double AXON_MAX_PLUS_SECONDS_PER_60_DEG_AT_6V = AxonMaxPlus.SECONDS_PER_60_DEG_AT_6V;
    public static final double HOOD_TRAVEL_DEG_PER_SECOND = AxonMaxPlus.DEGREES_PER_SECOND_AT_6V
            / TurretSubsystem.PITCH_SERVO_TO_TURRET_RATIO;
    public static final double MAX_YAW_DEG_PER_SECOND = AxonMaxPlus.DEGREES_PER_SECOND_AT_6V
            / TurretSubsystem.ROTATION_GEAR_RATIO;
    // Only soften command edges slightly. Velocity and acceleration have no software caps;
    // the servo's own controller determines physical motion. This is not a physical jerk limit.
    public static final double YAW_COMMAND_SMOOTHING_SECONDS = .005;
    public static final double YAW_COMMAND_SNAP_DEG = .001;
    public static final double ENCODER_CORRECTION_MARGIN_DEG = 3.0;
    public static final double ENCODER_CORRECTION_STOP_DEG = 1.5;
    public static final double ENCODER_CORRECTION_SETTLE_SECONDS = .25;
    public static final double ENCODER_CORRECTION_MAX_VELOCITY_DEG_PER_SECOND = 3.0;
    public static final double MAX_ENCODER_CORRECTION_DEG = 25.0;
    /**
     * A target jump larger than this between consecutive updates starts a new correction episode (cell switch,
     * long-way unwind, new target). It is about one 50 Hz loop of rated turret travel, so continuous tracking,
     * even through a capped chassis turn, keeps the learned offset instead of dropping it every few degrees.
     */
    public static final double ENCODER_CORRECTION_RESET_STEP_DEG = 10.0;
    public static final double MANUAL_STICK_DEADBAND = .08;
    public static final double MAX_UPDATE_SECONDS = .1;
    /**
     * Automatic aim leads the pose by this much: the time from the Pinpoint sample to the turret physically
     * reaching the command (hub servo frame, servo tracking lag, loop write timing). Chassis turn rate times
     * this delay was aim error. STARTING ESTIMATE, UNMEASURED: at least the ~10 ms average 50 Hz servo frame
     * plus write delay, so 25 ms cannot over-lead by more than it removes. Replace with Test: Auto Aim's
     * "Suggested aim lead" (spin at a steady rate) and record the measurement here.
     */
    public static final double AIM_LEAD_SECONDS = .025;
    public static final double MAX_AIM_LEAD_SECONDS = .15;
    private double estimatedHood, commandedYawDeg, lastTime = Double.NaN, lastHoodTime = Double.NaN;
    private double lastTargetDeg = Double.NaN, errorSince = Double.NaN, errorSign, correctionDeg;
    private boolean correctionApplied, correctionActive;
    private boolean encoderCorrectionEnabled;
    // Two short velocity filters soften manual acceleration changes; no physical jerk guarantee.
    public static final double MANUAL_VELOCITY_SMOOTHING_SECONDS = .01;
    private double manualVelocityStage1, manualVelocityStage2;
    private boolean competitionMotionActive;
    private boolean competitionWasAutomatic;

    public void setEncoderCorrectionEnabled(boolean enabled) {
        encoderCorrectionEnabled = enabled;
        correctionDeg = 0; correctionApplied = correctionActive = false; errorSince = Double.NaN;
    }
    public boolean isEncoderCorrectionEnabled() { return encoderCorrectionEnabled; }

    public void reset() {
        commandedYawDeg = TurretSubsystem.ROTATION_HOME_ANGLE_DEG;
        lastTime = lastHoodTime = Double.NaN;
        lastTargetDeg = Double.NaN; errorSince = Double.NaN;
        correctionDeg = 0; correctionApplied = correctionActive = false;
        manualVelocityStage1 = manualVelocityStage2 = 0;
        competitionMotionActive = false;
        competitionWasAutomatic = false;
    }

    /** Position command path without encoder correction; also advances the hood estimate. */
    public double updateYawCommand(double now, double targetYawDeg, double hoodCommandDeg) {
        updateHoodEstimate(now, hoodCommandDeg);
        return smoothYawCommand(now, targetYawDeg);
    }

    /**
     * Estimated hood travel toward the command in effect since the previous estimate (no feedback).
     * TurretSubsystem advances it with its sensor reads, before this loop's new hood command.
     */
    public void updateHoodEstimate(double now, double hoodCommandDeg) {
        double dt = elapsed(now, lastHoodTime);
        lastHoodTime = now;
        estimatedHood += RobotMath.clip(hoodCommandDeg - estimatedHood, HOOD_TRAVEL_DEG_PER_SECOND * dt);
    }

    private double smoothYawCommand(double now, double targetYawDeg) {
        double dt = elapsed(now, lastTime);
        lastTime = now;
        double target = TurretSubsystem.clampYawAngle(targetYawDeg);
        commandedYawDeg += (target - commandedYawDeg) * -Math.expm1(-dt / YAW_COMMAND_SMOOTHING_SECONDS);
        // Reach the exact target and then hold a constant PWM, without an asymptotic command tail.
        if (Math.abs(target - commandedYawDeg) <= YAW_COMMAND_SNAP_DEG) commandedYawDeg = target;
        return commandedYawDeg;
    }

    private static double elapsed(double now, double last) {
        return Double.isNaN(last) ? 0 : Math.max(0, Math.min(MAX_UPDATE_SECONDS, now - last));
    }

    /** Hysteretic position corrections plus the hood estimate, in one call (diagnostics and tests). */
    public double updateYawCommand(double now, double targetYawDeg, double measuredYawDeg,
                                   double measuredVelocityDegPerSecond, boolean homed, double hoodCommandDeg) {
        updateHoodEstimate(now, hoodCommandDeg);
        return updateYawCommand(now, targetYawDeg, measuredYawDeg, measuredVelocityDegPerSecond, homed);
    }

    /** Hysteretic position corrections, with a settled interval between bounded adjustments. */
    public double updateYawCommand(double now, double targetYawDeg, double measuredYawDeg,
                                   double measuredVelocityDegPerSecond, boolean homed) {
        double target = TurretSubsystem.clampYawAngle(targetYawDeg);
        // Continuous tracking keeps the learned offset; only a target jump (new episode) or lost home resets it.
        if (!Double.isFinite(lastTargetDeg) || Math.abs(target - lastTargetDeg) > ENCODER_CORRECTION_RESET_STEP_DEG || !homed) {
            correctionDeg = 0; correctionApplied = correctionActive = false; errorSince = Double.NaN;
        }
        lastTargetDeg = target;
        double error = target - measuredYawDeg;
        if (!encoderCorrectionEnabled || !homed || !RobotMath.finite(error, measuredVelocityDegPerSecond)) {
            correctionActive = false; errorSince = Double.NaN;
        } else {
            if (correctionActive && Math.abs(error) < ENCODER_CORRECTION_STOP_DEG) {
                // Retain the successful positional offset; removing it would move the turret again.
                correctionActive = false; errorSince = Double.NaN;
            }
            boolean needsCorrection = correctionActive || Math.abs(error) > ENCODER_CORRECTION_MARGIN_DEG;
            if (!needsCorrection || Math.abs(measuredVelocityDegPerSecond) > ENCODER_CORRECTION_MAX_VELOCITY_DEG_PER_SECOND) {
                errorSince = Double.NaN;
            } else if (Double.isNaN(errorSince) || Math.signum(error) != errorSign) {
                errorSince = now; errorSign = Math.signum(error);
            } else if (now - errorSince >= ENCODER_CORRECTION_SETTLE_SECONDS) {
                correctionActive = true;
                double offset = RobotMath.clip(correctionDeg + error, MAX_ENCODER_CORRECTION_DEG);
                correctionDeg = TurretSubsystem.clampYawAngle(target + offset) - target;
                correctionApplied = true;
                errorSince = Double.NaN; // Let the internal servo controller finish before checking again.
            }
        }
        return smoothYawCommand(now, target + correctionDeg);
    }
    public boolean isEncoderCorrectionActive() { return correctionActive; }
    public boolean hasAppliedEncoderCorrection() { return correctionApplied; }
    public double getEncoderCorrectionDegrees() { return correctionDeg; }

    public double hoodError(double commandDeg) { return Math.abs(commandDeg - estimatedHood); }
    public static double manualDelta(double input, double dt) {
        return input * MAX_YAW_DEG_PER_SECOND * Math.max(0, Math.min(dt, MAX_UPDATE_SECONDS));
    }
    /** Positive stick is clockwise. Release holds the commanded target, never the encoder reading. */
    public double manualYawTarget(double targetYawDeg, double stickX, double dt, boolean automatic) {
        if (automatic || Math.abs(stickX) <= MANUAL_STICK_DEADBAND) return targetYawDeg;
        return targetYawDeg - manualDelta(RobotMath.clip(stickX, 1), dt);
    }
    /** Competition only: linear steady speed, brief easing, and one encoder capture on release. */
    public double competitionManualYawTarget(double targetYawDeg, double measuredYawDeg,
                                             double stickX, double dt, boolean automatic) {
        boolean moving = Double.isFinite(stickX) && Math.abs(stickX) > MANUAL_STICK_DEADBAND;
        if (automatic) {
            competitionMotionActive = true;
            competitionWasAutomatic = true;
            manualVelocityStage1 = manualVelocityStage2 = 0;
            return targetYawDeg;
        }
        if (competitionWasAutomatic && Double.isFinite(measuredYawDeg)) {
            // A stick already held on auto release must also start from the measured heading.
            targetYawDeg = TurretSubsystem.clampYawAngle(measuredYawDeg);
            correctionDeg = 0; correctionApplied = correctionActive = false; errorSince = Double.NaN;
        }
        competitionWasAutomatic = false;
        if (!moving) {
            manualVelocityStage1 = manualVelocityStage2 = 0;
            if (competitionMotionActive && Double.isFinite(measuredYawDeg)) {
                competitionMotionActive = false;
                // Discard queued travel once. Subsequent encoder noise cannot move the hold target.
                double hold = TurretSubsystem.clampYawAngle(measuredYawDeg);
                correctionDeg = 0; correctionApplied = correctionActive = false; errorSince = Double.NaN;
                lastTargetDeg = hold;
                return hold;
            }
            return targetYawDeg;
        }
        competitionMotionActive = true;
        double seconds = Math.max(0, Math.min(MAX_UPDATE_SECONDS, dt));
        double velocity = -RobotMath.clip(stickX, 1) * MAX_YAW_DEG_PER_SECOND;
        // Analytic critically damped two-pole velocity filter, including its position integral.
        double a = manualVelocityStage1 - velocity;
        double b = manualVelocityStage2 - velocity;
        double tau = MANUAL_VELOCITY_SMOOTHING_SECONDS;
        double decay = Math.exp(-seconds / tau);
        double distance = velocity * seconds + b * tau * (1 - decay)
                + a * tau * (1 - (1 + seconds / tau) * decay);
        manualVelocityStage1 = velocity + a * decay;
        manualVelocityStage2 = velocity + (b + a * seconds / tau) * decay;
        return TurretSubsystem.clampYawAngle(targetYawDeg + distance);
    }
    public static double yawToRobotHeadingDegrees(double yawDeg) {
        return ((TurretSubsystem.ROTATION_HOME_ROBOT_HEADING_DEG - yawDeg) % 360 + 360) % 360;
    }
    /** Signed, continuous servo yaw: front=+90, right=0, back=-90, left=+/-180. */
    public static double robotHeadingToYawDegrees(double headingDeg) {
        return Math.toDegrees(RobotMath.wrap(Math.toRadians(TurretSubsystem.ROTATION_HOME_ROBOT_HEADING_DEG - headingDeg)));
    }
}
