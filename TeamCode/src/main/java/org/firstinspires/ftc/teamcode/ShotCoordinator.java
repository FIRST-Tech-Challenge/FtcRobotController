package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Sole owner of target/pose selection and automatic aiming; shared by TeleOp and autonomous. */
public final class ShotCoordinator {
    /**
     * AUTONOMOUS_RELOCALIZING: autonomous with TeleOp's tag pipeline (start-pose refinement toward a chosen
     * landmark, then periodic validated corrections of the aiming pose). AUTONOMOUS_ODOMETRY: no camera use.
     */
    public enum AimMode { DIAGNOSTIC, TELEOP_ODOMETRY, AUTONOMOUS_ODOMETRY, AUTONOMOUS_RELOCALIZING }
    public static final double MAX_OPENING_DISAGREEMENT_METERS = .08, MAX_OPENING_HEIGHT_ERROR_METERS = .1;
    public static final double FIELD_RELOCALIZATION_INTERVAL_SECONDS = .5;
    private final boolean fieldOdometryOnly;
    private final AimMode mode;
    private double lastFieldCorrection = Double.NEGATIVE_INFINITY;
    private MotionSample lastPoseSample;
    private double lastPoseUpdate = Double.NEGATIVE_INFINITY;
    private final TurretSubsystem turret;
    private final ShooterMount mount;
    private final ShotProfile profile;
    private final ShotPlanner planner;
    private final PoseHistory history = new PoseHistory();
    private final FieldRehomeRefinement fieldRehome = new FieldRehomeRefinement();
    private GoalTarget rehomeLandmark; // null: nearest cell of the selected alliance (TeleOp corner rehome)
    private final TurretLagEstimator lag = new TurretLagEstimator();
    private final GoalStateObserver goalState;
    private GoalTarget relativeTarget;
    private TagObservation trackingTag;
    private long previousReferenceId = Long.MIN_VALUE;
    private double trackingCapture = Double.NaN;
    private final FiringCoordinator firing;
    private GoalTarget target;
    private ShotPlanner.Solution solution = ShotPlanner.Solution.invalid("Not requested");
    private boolean previousAutomatic, previousHomed;
    private boolean presetValid, automaticTrackingReady, automaticHoodRpmAvailable;
    private double yawOffsetDeg, hoodOffsetDeg, rpmOffset;
    private MotionSample pose;
    private double lastFrame = Double.NEGATIVE_INFINITY;
    private double lastTrackingFrame = Double.NEGATIVE_INFINITY;
    private String visionDecision = "No camera observation processed";
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target) {
        this(turret, target, AimMode.DIAGNOSTIC);
    }
    /** Competition TeleOp uses fixed field targets; camera observations only correct the pose. */
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target, AimMode mode) {
        this(turret, target, ShootingConfiguration.MOUNT, ShootingConfiguration.PROFILE,
                mode == AimMode.TELEOP_ODOMETRY ? CompetitionFeedController.firing() : ShootingConfiguration.firing(),
                mode);
    }
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target, ShooterMount mount, ShotProfile profile,
                           FiringCoordinator firing) {
        this(turret, target, mount, profile, firing, AimMode.DIAGNOSTIC);
    }
    /** Explicit dependency configuration, including desktop-only synthetic measured profiles. */
    public ShotCoordinator(TurretSubsystem turret, GoalTarget target, ShooterMount mount, ShotProfile profile,
                            FiringCoordinator firing, AimMode mode) {
        this.mode = mode;
        this.fieldOdometryOnly = mode != AimMode.DIAGNOSTIC;
        this.turret = turret; this.target = target; this.mount = mount; this.profile = profile;
        goalState = new GoalStateObserver(mount,ShootingConfiguration.LOWERED_TAG_HEIGHT_METERS,ShootingConfiguration.LOWERED_TAG_PLANE_DEG); goalState.select(target.alliance,target.side);
        planner = new ShotPlanner(mount); this.firing = firing;
        turret.setTargetHiveCell(target.alliance, target.side);
    }
    public void select(GoalTarget next) {
        target = next; history.invalidateObservation(false); lastFrame = Double.NEGATIVE_INFINITY; solution = ShotPlanner.Solution.invalid("Target changed");
        relativeTarget = null; lastPoseSample = null;
        if (!fieldLocalizing()) lastFieldCorrection = Double.NEGATIVE_INFINITY;
        goalState.select(next.alliance,next.side);
        turret.setTargetHiveCell(next.alliance, next.side); turret.resetTrackingTrim();
        visionDecision = "Shot target changed; awaiting camera observation";
    }
    public void invalidateReference(double now) {
        if (fieldRehome.pending()) fieldRehome.cancel("reference invalidated");
        presetValid = false;
        history.reset(); relativeTarget = null; lastPoseSample = null; pose = null;
        solution = ShotPlanner.Solution.invalid("Reference changed");
        visionDecision = "Odometry reference changed; awaiting camera observation";
        firing.stop(now);
    }
    /** Camera frames correct the pose: TeleOp and relocalizing autonomous. */
    private boolean fieldLocalizing() { return mode == AimMode.TELEOP_ODOMETRY || mode == AimMode.AUTONOMOUS_RELOCALIZING; }
    public void requestFieldRehomeRefinement(double now) { requestFieldRehomeRefinement(now, null); }
    /** Refine an explicitly placed pose, pointing the turret at {@code landmark} (null: nearest selected cell). */
    public void requestFieldRehomeRefinement(double now, GoalTarget landmark) {
        if (!fieldLocalizing()) return;
        rehomeLandmark = landmark;
        fieldRehome.request(now);
    }
    public String fieldRehomeStatus() { return fieldRehome.status(); }
    public boolean fieldRehomePending() { return fieldRehome.pending(); }
    public void cancelFieldRehomeRefinement(String reason) {
        if (fieldRehome.pending()) fieldRehome.cancel(reason);
    }
    /** Offset changes must earn a fresh readiness interval, including during continuous fire. */
    public void setAutoAimAdjustments(double now, AutoAimAdjustments offsets) {
        if (yawOffsetDeg != offsets.yawDeg() || hoodOffsetDeg != offsets.hoodDeg() || rpmOffset != offsets.rpm()) {
            yawOffsetDeg = offsets.yawDeg(); hoodOffsetDeg = offsets.hoodDeg(); rpmOffset = offsets.rpm();
            automaticTrackingReady = false;
            firing.stop(now);
        }
    }
    /** Current checks only; the firing state machine still enforces the stable readiness interval. */
    public boolean automaticTrackingReady() { return automaticTrackingReady; }
    /**
     * Last update had a calibrated, aimable solution before operator trims, so automatic aim sets hood and RPM
     * and hood/RPM trims mean something. False with an uncalibrated profile: the shot uses manual hood/RPM.
     */
    public boolean automaticHoodRpmAvailable() { return automaticHoodRpmAvailable; }
    /** Call after one odometry update and one turret update. No second hardware reads. */
    public void update(double now, MotionSample sample, boolean automatic, boolean manualFeed,
                       boolean inhibit, boolean flywheelStop) {
        update(now, sample, automatic, automatic, manualFeed, inhibit, flywheelStop);
    }
    /** Autonomous can track throughout a route while requesting feed only in scoring states. */
    public void update(double now, MotionSample sample, boolean automatic, boolean automaticFeed,
                       boolean manualFeed, boolean inhibit, boolean flywheelStop) {
        update(now, sample, automatic, automaticFeed, manualFeed, inhibit, flywheelStop, true, true, true);
    }
    /** Autonomous output arbitration; TeleOp retains the original all-channel behavior. */
    public void update(double now, MotionSample sample, boolean automatic, boolean automaticFeed,
                       boolean manualFeed, boolean inhibit, boolean flywheelStop,
                       boolean controlYaw, boolean controlHood, boolean controlRpm) {
        presetValid = false;
        if (automatic != previousAutomatic) {
            turret.resetTrackingTrim(); lastTrackingFrame = Double.NEGATIVE_INFINITY;
        }
        previousAutomatic = automatic;
        updatePose(now, sample);
        if (mode == AimMode.TELEOP_ODOMETRY && pose != null && pose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS) && pose.fieldReferenced) {
            TurretSubsystem.HiveSide side = TeleOpFieldReference.nearestSide(pose.y, target.side);
            if (side != target.side) {
                select(TeleOpFieldReference.target(target.alliance, side));
                firing.stop(now); // A different cell must earn its own readiness interval.
            }
        }
        solution = !turret.isHomed() ? ShotPlanner.Solution.invalid("Turret: " + turret.getHomingStatus())
                : planner.solve(now, pose, relativeTarget != null ? relativeTarget : target, profile);
        // Before trims, so an out-of-range trim can still be adjusted back.
        automaticHoodRpmAvailable = solution.aimable;
        solution = solution.withOffsets(yawOffsetDeg, hoodOffsetDeg, rpmOffset);
        // Release / vision timing never blocks feeding: the current ShootingConfiguration values are used as-is.
        if (fieldRehome.pending() && solution.valid) solution = solution.inhibit("Corner Limelight refinement in progress");
        // Camera freshness/state gates apply only to the legacy diagnostics.
        if (!fieldOdometryOnly) {
            if (solution.aimable && !history.targetFresh(now)) solution = solution.inhibit("Target observation lost / stale: " + history.reason());
            if (solution.valid && !ShootingConfiguration.GOAL_OBSERVATION_VALIDATED) solution = solution.inhibit("Goal detector / pipeline UNVALIDATED");
            if (solution.valid && !goalState.raised(now)) solution = solution.inhibit("Goal state: " + goalState.reason());
        }
        if (automatic && solution.aimable) {
            // Servo gets the led bearing; readiness below still compares against the bearing now.
            if (controlYaw && !fieldRehome.pending()) turret.setTargetYawDegrees(turret.reachableYawTarget(
                    Math.toDegrees(solution.commandYaw)));
            if (controlHood) turret.setPitchAngleDegrees(solution.hoodDeg);
            if (controlRpm && !flywheelStop) turret.setShooterRpm(solution.rpm);
        } else if (fieldOdometryOnly && automatic && !fieldRehome.pending() && turret.isHomed() && pose != null
                && controlYaw && pose.fieldReferenced && pose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS)) {
            // Keep odometry yaw tracking available while RPM/hood calibration is pending.
            double degrees = Math.toDegrees(TeleOpFieldReference.yawRadians(pose.led(planner.leadSeconds()), target, mount)) + yawOffsetDeg;
            turret.setTargetYawDegrees(turret.reachableYawTarget(degrees));
        } else if (!fieldOdometryOnly && automatic && controlYaw && turret.isHomed() && turret.getVisionTag() != null
                && turret.getVisionResult().getTimestamp() > lastTrackingFrame) {
            // Acquire the selected hive before shot calibration is available; this does not enable firing.
            // Vision selection already filters alliance, hive side, finite tx and stale frames.
            double compensated = trackingTag != null ? history.delayedYaw3d(now,trackingCapture,trackingTag,mount,planner.leadSeconds())
                    : history.delayedYaw(now,trackingCapture,Math.toRadians(turret.getVisionTag().getTargetXDegrees()),
                            Math.toRadians(turret.getVisionTag().getTargetYDegrees()),mount,planner.leadSeconds());
            if (Double.isFinite(compensated)) {
                turret.setTargetYawDegrees(turret.reachableYawTarget(Math.toDegrees(compensated)));
            }
            lastTrackingFrame = turret.getVisionResult().getTimestamp();
        }
        if (flywheelStop) turret.stopShooter();
        // Measure residual turret lag against the bearing now (telemetry only; see TurretLagEstimator).
        if (automatic && controlYaw && turret.isHomed() && !fieldRehome.pending() && pose != null
                && pose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS) && (solution.aimable || (fieldOdometryOnly && pose.fieldReferenced))) {
            double bearingNow = solution.aimable ? Math.toDegrees(solution.yaw)
                    : Math.toDegrees(TeleOpFieldReference.yawRadians(pose, target, mount)) + yawOffsetDeg;
            lag.update(now, turret.reachableYawTarget(bearingNow),
                    turret.getEncoderAngleDegrees(), turret.getEncoderVelocityDegPerSec(), Math.toDegrees(pose.omega));
        }
        double yawError = solution.valid ? Math.abs(Math.toDegrees(solution.yaw) - turret.getEncoderAngleDegrees()) : Double.POSITIVE_INFINITY;
        boolean angleReady = ShotReadiness.angleReady(yawError);
        boolean tight = angleReady && turret.isHoodTracking() && turret.isShooterAtSpeed();
        boolean loose = angleReady && turret.isHoodTracking()
                && turret.getTargetShooterRpm() > 0 && Math.abs(turret.getShooterCurrentRpm() - solution.rpm)
                <= TurretSubsystem.SHOOTER_LOOSE_READY_TOLERANCE_RPM;
        automaticTrackingReady = automatic && solution.valid && tight && !inhibit && !flywheelStop;
        firing.update(now, automatic && automaticFeed, manualFeed, inhibit, flywheelStop, solution.valid, tight, loose, solution.reason);
        turret.setTrackingStatus(solution.reason);
    }
    /** Fixed presets share the same feed state machine; unmeasured presets can only prepare. */
    public void updatePreset(double now, MotionSample sample, AutonomousShotPresets.Preset preset,
                             boolean automaticFeed, boolean manualFeed, boolean inhibit) {
        updatePose(now, sample);
        automaticHoodRpmAvailable = false;
        boolean valid = preset.measured && turret.isHomed()
                && sample != null && sample.fieldReferenced && sample.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS);
        presetValid = valid;
        String reason = !preset.measured ? "Preset UNMEASURED: " + preset.name
                : !valid ? "Preset requires home and fresh field reference" : "Preset ready";
        solution = ShotPlanner.Solution.invalid(reason);
        boolean angle = ShotReadiness.angleReady(Math.abs(preset.yawDeg - turret.getEncoderAngleDegrees()));
        boolean tight = angle && turret.isHoodTracking() && turret.isShooterAtSpeed();
        boolean loose = angle && turret.isHoodTracking() && Math.abs(turret.getShooterCurrentRpm() - preset.outputRpm)
                <= TurretSubsystem.SHOOTER_LOOSE_READY_TOLERANCE_RPM;
        firing.update(now, automaticFeed, manualFeed, inhibit, false, valid, tight, loose, reason);
        turret.setTrackingStatus(reason);
    }
    static double visualTrackingYawDegrees(double measuredYawDeg, double txDeg, double cameraYawDeg) {
        // tx is right-positive; internal turret yaw and camera mounting yaw are left-positive.
        // Dead-zone hold and long-way unwind are shared with odometry aiming.
        return TurretSubsystem.reachableYawDegrees(measuredYawDeg + cameraYawDeg - txDeg, measuredYawDeg);
    }
    public void updatePose(double now, MotionSample sample) {
        if (sample == lastPoseSample && now == lastPoseUpdate && turret.isHomed() == previousHomed) return;
        lastPoseSample = sample; lastPoseUpdate = now;
        if (turret.isHomed() != previousHomed) {
            history.invalidateObservation(true); lastFrame = Double.NEGATIVE_INFINITY;
            turret.resetTrackingTrim(); relativeTarget = null; goalState.invalidate("Turret reference changed"); previousHomed = turret.isHomed();
        }
        if (sample == null) {
            pose = null; relativeTarget = null; goalState.invalidate("No odometry");
            visionDecision = "No Pinpoint sample"; return;
        }
        if (sample.referenceId != previousReferenceId) {
            firing.stop(now);
            history.reset(); relativeTarget = null; goalState.invalidate("Odometry reference changed");
            lastFieldCorrection = Double.NEGATIVE_INFINITY;
            previousReferenceId = sample.referenceId;
        }
        history.add(sample, Math.toRadians(turret.getEncoderAngleDegrees()));
        if (mode == AimMode.AUTONOMOUS_ODOMETRY) {
            // Routes and shots share odometry. No mid-route camera correction or direct visual aim.
            pose = history.current(now);
            visionDecision = "Camera corrections disabled during autonomous route";
            return;
        }
        LLResult result = turret.getVisionResult();
        LimelightObservation observation = turret.getVisionObservation();
        if (observation == null) observation = new LimelightObservation(now,System.currentTimeMillis(),result);
        if (fieldLocalizing()) {
            updateFieldLocalization(now,observation);
            pose = history.current(now);
            return;
        }
        trackingCapture = observation.captureTime; trackingTag = null;
        for (TagObservation tag : observation.tags) {
            if (TurretSubsystem.getHiveSideForTag(target.alliance,tag.id) == target.side && tag.usable()
                    && (trackingTag == null || tag.uncertaintyMeters()<trackingTag.uncertaintyMeters())) trackingTag = tag;
        }
        if (!fieldOdometryOnly) {
            goalState.update(now,observation.frame,observation.captureTime,observation.tags,ShootingConfiguration.LEVEL_CHASSIS_ASSUMED);
            visionDecision = observation.rejectionReason != null ? observation.rejectionReason : goalState.reason();
            if (!goalState.raised(now)) relativeTarget = null;
        }
        if (result != null && turret.isHomed() && result.getTimestamp() > lastFrame) {
            lastFrame = result.getTimestamp();
            if (fieldOdometryOnly || goalState.raised(now)) {
                TagObservation best = trackingTag;
                if (best != null && target.scoreable() && sample.fieldReferenced) {
                    boolean correctionAllowed = !fieldOdometryOnly
                            || (ShootingConfiguration.FIELD_RELOCALIZATION_ENABLED
                            && now - lastFieldCorrection >= FIELD_RELOCALIZATION_INTERVAL_SECONDS);
                    if (correctionAllowed) {
                        if (history.observe3d(now,observation.frame,observation.captureTime,best,target,mount)) {
                            lastFieldCorrection = now;
                            visionDecision = history.reason();
                        } else {
                            visionDecision = history.reason();
                            relativeTarget = null; history.invalidateObservation(false);
                        }
                    }
                } else if (!fieldOdometryOnly && best != null) observeOpening(observation);
            }
        }
        pose = history.current(now);
    }
    private void updateFieldLocalization(double now, LimelightObservation observation) {
        MotionSample current = history.current(now);
        if (fieldRehome.pending()) {
            fieldRehome.expire(now);
            if (!ShootingConfiguration.FIELD_RELOCALIZATION_ENABLED || !ShootingConfiguration.LEVEL_CHASSIS_ASSUMED
                    || !mount.measured) fieldRehome.cancel("vision geometry/configuration unavailable");
            if (!fieldRehome.pending() || !turret.isHomed() || current == null
                    || !current.fresh(now,ShotPlanner.MAX_SAMPLE_AGE_SECONDS)) {
                visionDecision = !fieldRehome.pending() ? fieldRehome.status()
                        : !turret.isHomed() ? "Turret not homed: " + turret.getHomingStatus()
                        : "Pinpoint sample unavailable / not ready / stale";
                return;
            }
            if (fieldRehome.update(now,observation.frame,observation.captureTime,observation.tags,history,mount))
                lastFieldCorrection = now;
            if (fieldRehome.pending()) {
                GoalTarget landmark = rehomeLandmark != null ? rehomeLandmark : TeleOpFieldReference.target(target.alliance,
                        TeleOpFieldReference.nearestSide(current.y,target.side));
                double yaw = Math.toDegrees(TeleOpFieldReference.yawRadians(current.led(planner.leadSeconds()),landmark,mount));
                turret.setTargetYawDegrees(turret.reachableYawTarget(yaw));
            }
            if (Double.isFinite(observation.frame)) lastFrame = observation.frame;
            visionDecision = fieldRehome.status();
            return;
        }
        if (!ShootingConfiguration.FIELD_RELOCALIZATION_ENABLED) { visionDecision = "Field relocalization disabled by configuration"; return; }
        if (!ShootingConfiguration.LEVEL_CHASSIS_ASSUMED) { visionDecision = "Requires supported level chassis"; return; }
        if (!mount.measured) { visionDecision = "Camera mount unmeasured"; return; }
        if (!turret.isHomed()) { visionDecision = "Turret not homed: " + turret.getHomingStatus(); return; }
        if (current == null || !current.fresh(now,ShotPlanner.MAX_SAMPLE_AGE_SECONDS)) {
            visionDecision = "Pinpoint sample unavailable / not ready / stale"; return;
        }
        if (observation.rejectionReason != null) { visionDecision = observation.rejectionReason; return; }
        if (now < observation.captureTime || now-observation.captureTime > PoseHistory.MAX_OBSERVATION_AGE_SECONDS) {
            visionDecision = "Stale / future camera capture"; return;
        }
        if (observation.frame <= lastFrame) return; // Retain the last decision while polling the same frame.
        lastFrame = observation.frame;
        // Fuse every accepted tag in the frame; a single "best" tag changed frame to frame and its
        // per-tag noise moved the aim point.
        java.util.List<TagObservation> accepted = new java.util.ArrayList<>();
        java.util.List<GoalTarget> landmarks = new java.util.ArrayList<>();
        String rejected = "No mapped hive tags in camera frame";
        for (TagObservation tag : observation.tags) {
            String rejection = TeleOpFieldReference.raisedTagRejectionReason(tag,mount);
            if (rejection != null) { rejected = rejection; continue; }
            accepted.add(tag); landmarks.add(TeleOpFieldReference.targetForTag(tag.id));
        }
        if (accepted.isEmpty()) { visionDecision = rejected; return; }
        boolean fused = current.fieldReferenced
                // Odometry owns the pose; the camera only corrects consistent drift beyond the deadband.
                ? history.checkDrift3d(now,observation.frame,observation.captureTime,accepted,landmarks,mount)
                : history.acquireFieldReference3d(now,observation.frame,observation.captureTime,accepted,landmarks,mount);
        if (fused) lastFieldCorrection = now;
        visionDecision = history.reason();
    }
    private void observeOpening(LimelightObservation observation) {
        PoseHistory.Entry capture = history.at(observation.captureTime);
        if (capture == null || !ShootingConfiguration.OPENING.verified) {
            relativeTarget = null;
            visionDecision = capture == null ? "Missing capture-time Pinpoint / turret history" : "Tag-to-opening transform unverified";
            return;
        }
        double x = 0, y = 0, z = 0, uncertainty = .025;
        int count = 0;
        java.util.List<GoalTarget.Tag> tags = new java.util.ArrayList<>();
        for (TagObservation tag : observation.tags) {
            if (TurretSubsystem.getHiveSideForTag(target.alliance,tag.id) != target.side) continue;
            double[] point = ShootingConfiguration.OPENING.opening(tag,mount,capture.pose,capture.turretYaw);
            if (point == null) { relativeTarget = null; return; }
            if (count > 0 && Math.sqrt(Math.pow(point[0]-x/count,2)+Math.pow(point[1]-y/count,2)+Math.pow(point[2]-z/count,2)) > MAX_OPENING_DISAGREEMENT_METERS) {
                goalState.invalidate("Opening reprojections disagree"); relativeTarget = null; return;
            }
            x += point[0]; y += point[1]; z += point[2]; count++;
            uncertainty = Math.max(uncertainty,tag.uncertaintyMeters());
            double[] level = tag.levelPosition(mount);
            org.firstinspires.ftc.teamcode.RobotMath.Vec camera = mount.cameraPosition(capture.pose,capture.turretYaw);
            org.firstinspires.ftc.teamcode.RobotMath.Vec position = new org.firstinspires.ftc.teamcode.RobotMath.Vec(level[0],level[1])
                    .rotate(capture.pose.heading+mount.zeroYaw+capture.turretYaw+mount.cameraYaw).plus(camera);
            tags.add(new GoalTarget.Tag(tag.id,position.x,position.y,mount.cameraHeight+level[2]));
        }
        if (count == 0) { relativeTarget = null; return; }
        x /= count; y /= count; z /= count;
        if (Math.abs(z-ShootingConfiguration.HIGH_GOAL_CENTER_HEIGHT_METERS) > MAX_OPENING_HEIGHT_ERROR_METERS
                || (relativeTarget != null && Math.hypot(x-relativeTarget.x,y-relativeTarget.y) > PoseHistory.MAX_JUMP_METERS)) {
            goalState.invalidate("Implausible observed opening correction"); relativeTarget = null; return;
        }
        relativeTarget = new GoalTarget(target.alliance,target.side,GoalTarget.HiveState.RAISED,x,y,z,true,
                ShootingConfiguration.OPENING.identity,tags.toArray(new GoalTarget.Tag[0])).inLocalFrame();
        history.acceptRelative(observation.captureTime,uncertainty);
        visionDecision = "Accepted target-relative opening";
    }
    public FiringCoordinator firing() { return firing; }
    public TurretLagEstimator lag() { return lag; }
    public boolean validForFeeding() { return presetValid || solution.valid; }
    public ShotPlanner.Solution solution() { return solution; }
    public MotionSample pose() { return pose; }
    public String visionDecision() { return visionDecision; }
    /** Shown only around a large camera/odometry disagreement or a re-localization (TeleOp odometry aiming). */
    private void addPoseCheckTelemetry(Telemetry telemetry, double now) {
        String check = fieldLocalizing() ? history.largeCorrectionStatus(now) : null;
        if (check != null) telemetry.addData("Camera pose check", check);
    }
    private String localizationStatus(double now) {
        return pose != null && pose.fieldReferenced
                ? pose.fresh(now,ShotPlanner.MAX_SAMPLE_AGE_SECONDS) ? "LOCALIZED" : "FIELD REFERENCE RETAINED; ODOMETRY NOT READY / STALE"
                : "WAITING: " + visionDecision;
    }

    /**
     * Compact match view: localization, the latest corner-rehome result, and auto-aim trims while
     * aiming or whenever a trim is still applied.
     */
    public void addDriverTelemetry(Telemetry telemetry, double now, boolean aiming) {
        if (fieldOdometryOnly) telemetry.addData("Field localization", localizationStatus(now));
        addPoseCheckTelemetry(telemetry, now);
        if (fieldLocalizing() && !FieldRehomeRefinement.NOT_REQUESTED.equals(fieldRehome.status()))
            telemetry.addData("Field rehome refinement", fieldRehome.status());
        if (aiming || yawOffsetDeg != 0 || hoodOffsetDeg != 0 || rpmOffset != 0)
            telemetry.addData("Auto trims (yaw deg / hood deg / RPM)", "%.1f / %.1f / %.0f", yawOffsetDeg, hoodOffsetDeg, rpmOffset);
    }

    public void addTelemetry(Telemetry telemetry, double now) {
        telemetry.addData("Aim reference", mode == AimMode.AUTONOMOUS_ODOMETRY ? "FIELD ODOMETRY | fixed route target"
                : mode == AimMode.AUTONOMOUS_RELOCALIZING ? "FIELD ODOMETRY | fixed route target | periodic validated vision corrections"
                : fieldOdometryOnly ? "FIELD ODOMETRY | periodic validated vision corrections" : "Field / target-relative diagnostics");
        telemetry.addData("Shot target", relativeTarget != null ? relativeTarget.identity() : target.identity());
        if (fieldOdometryOnly) {
            telemetry.addData("Hive state", "Driver responsibility; no automatic state tracking");
            telemetry.addData("Vision relocalization", mode == AimMode.AUTONOMOUS_ODOMETRY ? "Disabled during autonomous route" : target.tags.isEmpty()
                    ? "No tag field map" : ShootingConfiguration.FIELD_RELOCALIZATION_ENABLED
                    ? "Enabled | nominal TU06 map | " + history.reason() : "Disabled by configuration");
            telemetry.addData("Field localization", localizationStatus(now));
            addPoseCheckTelemetry(telemetry, now);
            if (fieldLocalizing()) telemetry.addData("Field rehome refinement", fieldRehome.status());
            if (pose != null && pose.fieldReferenced) {
                telemetry.addData("Aiming field pose (m, deg)", "%.3f, %.3f | %.1f", pose.x, pose.y, Math.toDegrees(pose.heading));
                org.firstinspires.ftc.teamcode.RobotMath.Vec pivot = new org.firstinspires.ftc.teamcode.RobotMath.Vec(pose.x, pose.y).plus(mount.pivot.rotate(pose.heading));
                telemetry.addData("Odometry target range / yaw (m, deg)", "%.3f | %.1f",
                        Math.hypot(target.x - pivot.x, target.y - pivot.y),
                        Math.toDegrees(TeleOpFieldReference.yawRadians(pose, target, mount)));
            }
        } else {
            telemetry.addData("Observed cell state", "%s | %s",goalState.state(now),goalState.reason());
            telemetry.addData("Tag height / plane (m, deg)", "%.3f | %.1f | age %.3f s",goalState.heightMeters(),goalState.planeDegrees(),goalState.age(now));
        }
        telemetry.addData("Shot profile", "%s | %s", profile.geometryId, profile.calibrated ? "CALIBRATED" : "UNCALIBRATED EXAMPLE");
        telemetry.addData("Shot solution", solution.reason);
        telemetry.addData("Auto offsets (yaw deg / hood deg / RPM)", "%.1f / %.1f / %.0f", yawOffsetDeg, hoodOffsetDeg, rpmOffset);
        telemetry.addData("Accepted vision pose age (s)", history.observationAge(now));
        telemetry.addData("Vision decision", visionDecision);
        telemetry.addData("Pose fusion decision", history.reason());
        if (pose != null) telemetry.addData("Odometry-frame velocity (m/s, rad/s)", "%.2f, %.2f | %.2f", pose.vx, pose.vy, pose.omega);
        if (pose != null) telemetry.addData("Pose uncertainty estimate (m)", "%.3f | no drift assumed; no feed cutoff", pose.uncertainty);
        telemetry.addData("Current aim yaw / led command / range (deg, deg, m)", "%.2f | %.2f | %.3f",
                Math.toDegrees(solution.yaw), Math.toDegrees(solution.commandYaw), solution.effectiveRange);
        lag.addTelemetry(telemetry, planner.leadSeconds());
        telemetry.addData("Auto feed readiness", "Angle within 3 deg + hood/RPM; no motion gate");
        telemetry.addData("Firing", "%s | %s | committed ball possible: %s", firing.state(), firing.reason(), firing.ballMayBeCommitted());
        telemetry.addData("Timed feed attempts (not balls)", firing.feedAttempts());
        telemetry.addData("Timed shooting (s)", "gate %.2f | feed %.2f | recovery %.2f",
                firing.gateTravelSeconds, firing.feedWindowSeconds, firing.recoverySeconds);
    }
}
