package org.firstinspires.ftc.teamcode;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.List;
import java.util.Deque;
import org.firstinspires.ftc.teamcode.RobotMath.Vec;

/** Timestamped pose/turret history with a local-to-field transform and translation corrections. */
public final class PoseHistory {
    public static final double HISTORY_SECONDS = 1.0, MAX_OBSERVATION_AGE_SECONDS = .35;
    public static final double MAX_JUMP_METERS = .5, MAX_TARGET_AGE_SECONDS = .75;
    public static final int ACQUISITION_FRAMES = 3;
    public static final double ACQUISITION_POSITION_TOLERANCE_METERS = .15;
    public static final double ACQUISITION_HEADING_TOLERANCE_RAD = Math.toRadians(8);
    public static final double TAG_PLANE_TOLERANCE_DEG = 10;
    /** Fraction of each accepted multi-tag frame applied to position and heading. */
    public static final double CORRECTION_GAIN = .35;
    /** A later frame may nudge heading by at most this much error; larger disagreement is rejected. */
    public static final double MAX_HEADING_CORRECTION_RAD = Math.toRadians(8);
    /** Tags within one frame must agree this closely on robot heading and position. */
    public static final double FRAME_HEADING_AGREEMENT_RAD = Math.toRadians(8), FRAME_POSITION_AGREEMENT_METERS = .10;
    /**
     * A rotating camera turns capture-latency error into a pose error. Fuse only frames whose turret
     * and chassis rotation stayed below this rate (plus encoder noise) around capture and since.
     */
    public static final double MAX_FUSION_TURN_RADIANS_PER_SECOND = Math.toRadians(20);
    public static final double FUSION_SETTLE_SECONDS = .10, FUSION_ANGLE_NOISE_RADIANS = Math.toRadians(.5);
    /**
     * Drift-only relocalization after acquisition: Pinpoint owns the pose. The camera compares the last
     * DRIFT_FRAMES settled multi-tag frames (within DRIFT_WINDOW_SECONDS) against odometry and changes the
     * pose only when their consistent average disagrees by more than the deadband; that correction is
     * blended in over DRIFT_BLEND_SECONDS so the turret does not snap.
     */
    public static final int DRIFT_FRAMES = 5;
    public static final double DRIFT_WINDOW_SECONDS = 1.0, DRIFT_BLEND_SECONDS = .4;
    public static final double DRIFT_POSITION_DEADBAND_METERS = .04, DRIFT_HEADING_DEADBAND_RAD = Math.toRadians(1.5);
    public static final double DRIFT_FRAME_AGREEMENT_METERS = .04, DRIFT_FRAME_AGREEMENT_RAD = Math.toRadians(1.5);
    /**
     * A disagreement beyond MAX_JUMP_METERS / MAX_HEADING_CORRECTION_RAD usually means the odometry reference itself
     * is wrong (pods slipped in a collision, a corner rehome or autonomous handoff from the wrong place). It
     * re-localizes only after at least RELOCALIZE_FRAMES settled frames, each fusing at least RELOCALIZE_MIN_TAGS
     * agreeing tags, spanning RELOCALIZE_MIN_SPAN_SECONDS inside RELOCALIZE_WINDOW_SECONDS, all imply the same rigid
     * correction (drift agreement limits) and put the robot on the field. Any frame that agrees with odometry, an
     * inconsistent frame set or a single-tag frame restarts the count. The correction blends in like drift.
     */
    public static final int RELOCALIZE_FRAMES = 10, RELOCALIZE_MIN_TAGS = 2;
    public static final double RELOCALIZE_MIN_SPAN_SECONDS = .5, RELOCALIZE_WINDOW_SECONDS = 1.0;
    /** How long the driver view keeps showing a large disagreement / a completed re-localization. */
    public static final double DISAGREEMENT_DISPLAY_SECONDS = 1.0, RELOCALIZED_DISPLAY_SECONDS = 3.0;
    public static final class Entry {
        public final MotionSample pose;
        public final double turretYaw;
        Entry(MotionSample pose, double turretYaw) { this.pose = pose; this.turretYaw = turretYaw; }
    }
    private final Deque<Entry> history = new ArrayDeque<>();
    private double correctionX, correctionY, lastFrame = Double.NEGATIVE_INFINITY;
    private double correctionYaw, candidateX, candidateY, candidateYaw, candidateTime;
    private double candidateSumX, candidateSumY, candidateSumSin, candidateSumCos, candidateLocalX, candidateLocalY;
    private int candidateFrames;
    private boolean visionFieldReference;
    private double lastObservation = Double.NEGATIVE_INFINITY;
    /** Settled camera-minus-odometry samples: {capture, headingError, dx, dy, pivotX, pivotY}. */
    private final Deque<double[]> driftFrames = new ArrayDeque<>();
    /** Large-disagreement frames: {capture, cameraHeading, cameraX, cameraY, odometryHeading, odometryX, odometryY}. */
    private final Deque<double[]> largeFrames = new ArrayDeque<>();
    private double largeSeenAt = Double.NEGATIVE_INFINITY, relocalizedAt = Double.NEGATIVE_INFINITY;
    private String largeStatus = "", relocalizedSummary = "";
    private double blendYaw, blendX, blendY, blendPivotX, blendPivotY, blendRemaining, blendTime;
    private double acceptedUncertainty = Double.NaN;
    private String reason = "No target observation";
    public void reset() {
        history.clear(); acceptedUncertainty = Double.NaN; correctionX = correctionY = 0; lastFrame = lastObservation = Double.NEGATIVE_INFINITY;
        correctionYaw = 0; visionFieldReference = false; candidateFrames = 0;
        driftFrames.clear(); largeFrames.clear(); blendRemaining = 0;
        largeSeenAt = relocalizedAt = Double.NEGATIVE_INFINITY;
        reason = "Reference / target changed";
    }
    /** A new target or encoder home does not change the established local-to-field transform. */
    public void invalidateObservation(boolean clearHistory) {
        if (clearHistory) history.clear();
        candidateFrames = 0; driftFrames.clear(); largeFrames.clear();
        lastFrame = lastObservation = Double.NEGATIVE_INFINITY;
        if (!visionFieldReference) acceptedUncertainty = Double.NaN;
        reason = "Target / turret reference changed";
    }
    public void add(MotionSample sample, double turretYaw) {
        if (!sample.fresh(sample.time, .1) || !Double.isFinite(turretYaw)) { history.clear(); return; }
        if (!history.isEmpty() && sample.referenceId != history.getLast().pose.referenceId) reset();
        if (!history.isEmpty() && sample.time <= history.getLast().pose.time) return;
        history.addLast(new Entry(sample, turretYaw));
        applyBlend(sample.time);
        while (history.size() > 2 && (sample.time - history.getFirst().pose.time > HISTORY_SECONDS || history.size() > 256)) history.removeFirst();
    }
    public Entry at(double time) {
        Entry previous = null;
        for (Entry next : history) {
            if (next.pose.time == time) return corrected(next);
            if (next.pose.time > time) {
                if (previous == null || next.pose.time - previous.pose.time > .1) return null;
                double t = (time - previous.pose.time) / (next.pose.time - previous.pose.time);
                MotionSample a = previous.pose, b = next.pose;
                MotionSample p = new MotionSample(time, a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                        a.heading + t * RobotMath.wrap(b.heading - a.heading), a.vx + t * (b.vx - a.vx),
                        a.vy + t * (b.vy - a.vy), a.omega + t * (b.omega - a.omega),
                        Math.max(a.acceleration, b.acceleration), Math.max(a.angularAcceleration, b.angularAcceleration), Math.max(a.uncertainty, b.uncertainty),
                        a.ready && b.ready, a.fieldReferenced && b.fieldReferenced, b.status).withReference(b.referenceId);
                return corrected(new Entry(p, previous.turretYaw + t * RobotMath.wrap(next.turretYaw - previous.turretYaw)));
            }
            previous = next;
        }
        return null;
    }
    private MotionSample corrected(MotionSample p, double uncertainty) {
        Vec position = new Vec(p.x,p.y).rotate(correctionYaw).plus(new Vec(correctionX,correctionY));
        Vec velocity = new Vec(p.vx,p.vy).rotate(correctionYaw);
        return new MotionSample(p.time,position.x,position.y,RobotMath.wrap(p.heading+correctionYaw),
                velocity.x,velocity.y,p.omega,p.acceleration,p.angularAcceleration,uncertainty,
                p.ready,p.fieldReferenced || visionFieldReference,p.status).withReference(p.referenceId);
    }
    private Entry corrected(Entry e) { return new Entry(corrected(e.pose,e.pose.uncertainty), e.turretYaw); }
    public MotionSample current(double now) {
        if (history.isEmpty()) return null;
        MotionSample p = history.getLast().pose;
        // No assumed drift or missing-camera penalty. Observation age still controls frame use.
        double uncertainty = Double.isFinite(acceptedUncertainty) ? acceptedUncertainty : p.uncertainty;
        return corrected(p, uncertainty);
    }
    /** Apply a validated corner/vision fusion result without changing heading or differentiating a reset. */
    boolean refineFieldTranslation(double dx, double dy, double uncertainty, double capture) {
        if (history.isEmpty() || !history.getLast().pose.fieldReferenced
                || !RobotMath.finite(dx,dy,uncertainty,capture) || uncertainty <= 0
                || Math.hypot(dx,dy) > FieldRehomeRefinement.MAX_CORRECTION_METERS) return false;
        correctionX += dx; correctionY += dy; acceptedUncertainty = uncertainty; lastObservation = capture;
        reason = "Corner reference refined with consistent Limelight frames";
        return true;
    }
    /** Single-tag form kept for callers with one observation; see the multi-tag overload. */
    public boolean acquireFieldReference3d(double now, double frame, double capture, TagObservation tag,
                                           GoalTarget target, ShooterMount mount) {
        return acquireFieldReference3d(now,frame,capture,tag == null ? null : Collections.singletonList(tag),
                Collections.singletonList(target),mount);
    }
    /**
     * Bootstrap position and heading from every accepted raised tag in one frame. Heading comes from
     * each tag's +X edge, which stays horizontal on the 30-deg cell (2.5x steadier than the steep
     * outward normal on the physical 2026-11-05 frame); position and heading are averaged over tags
     * and over the ACQUISITION_FRAMES consistent frames.
     */
    public boolean acquireFieldReference3d(double now, double frame, double capture, List<TagObservation> tags,
                                           List<GoalTarget> targets, ShooterMount mount) {
        if (tags == null || tags.isEmpty() || tags.contains(null)) return reject("Missing metric acquisition pose");
        for (TagObservation tag : tags) {
            String rejection = tag.rejectionReason();
            if (rejection != null) return reject("Tag " + tag.id + ": " + rejection);
        }
        if (!RobotMath.finite(now,frame,capture) || frame <= lastFrame)
            return reject("Invalid / duplicate acquisition frame");
        lastFrame = frame;
        if (now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS) return rejectAcquisition("Stale acquisition frame");
        Entry e = at(capture);
        if (e == null || e.pose.fieldReferenced || mapRejection(tags,targets,mount) != null)
            return rejectAcquisition("Missing acquisition history / field map");
        if (geometryRejection(tags,targets,mount,true) != null) return rejectAcquisition("Tag is not in nominal raised geometry");
        String motion = fusionMotionRejection(capture);
        if (motion != null) return rejectAcquisition(motion);
        double[] fix = new double[4];
        String disagreement = fuseFrame(tags,targets,e,mount,fix);
        if (disagreement != null) return rejectAcquisition(disagreement);
        Vec position = new Vec(fix[1],fix[2]), local = new Vec(e.pose.x,e.pose.y);
        if (Math.abs(position.x) > TeleOpFieldReference.FIELD_HALF_WIDTH_METERS
                || Math.abs(position.y) > TeleOpFieldReference.FIELD_HALF_WIDTH_METERS)
            return rejectAcquisition("Acquired pose outside field");
        double yaw = RobotMath.wrap(fix[0]-e.pose.heading);
        if (candidateFrames > 0 && capture-candidateTime <= MAX_OBSERVATION_AGE_SECONDS) {
            Vec translation = position.minus(local.rotate(candidateYaw));
            if (Math.hypot(translation.x-candidateX,translation.y-candidateY) > ACQUISITION_POSITION_TOLERANCE_METERS
                    || Math.abs(RobotMath.wrap(yaw-candidateYaw)) > ACQUISITION_HEADING_TOLERANCE_RAD) candidateFrames = 0;
        } else candidateFrames = 0;
        if (candidateFrames == 0) {
            candidateSumX = candidateSumY = candidateSumSin = candidateSumCos = 0;
            candidateLocalX = candidateLocalY = 0;
        }
        candidateTime = capture; candidateFrames++;
        candidateSumSin += Math.sin(yaw); candidateSumCos += Math.cos(yaw);
        candidateSumX += position.x; candidateSumY += position.y;
        candidateLocalX += local.x; candidateLocalY += local.y;
        candidateYaw = Math.atan2(candidateSumSin,candidateSumCos);
        Vec mean = new Vec(candidateSumX/candidateFrames,candidateSumY/candidateFrames)
                .minus(new Vec(candidateLocalX/candidateFrames,candidateLocalY/candidateFrames).rotate(candidateYaw));
        candidateX = mean.x; candidateY = mean.y;
        if (candidateFrames < ACQUISITION_FRAMES) return reject("Acquiring field reference " + candidateFrames + "/" + ACQUISITION_FRAMES);
        correctionYaw = candidateYaw; correctionX = candidateX; correctionY = candidateY; visionFieldReference = true;
        driftFrames.clear(); blendRemaining = 0;
        acceptRelative(capture,fix[3]); reason = "Field position / heading acquired from tags " + ids(tags);
        return true;
    }
    /** Null when every tag has a field-map entry in its landmark target. */
    private static String mapRejection(List<TagObservation> tags, List<GoalTarget> targets, ShooterMount mount) {
        if (targets == null || targets.size() != tags.size() || !mount.measured) return "Missing field map";
        for (int i = 0; i < tags.size(); i++) {
            GoalTarget target = targets.get(i);
            if (target == null || !target.scoreable() || target.frame != GoalTarget.Frame.FIELD
                    || target.tags.get(tags.get(i).id) == null) return "Missing field map for tag " + tags.get(i).id;
        }
        return null;
    }
    private static String geometryRejection(List<TagObservation> tags, List<GoalTarget> targets, ShooterMount mount, boolean checkPlane) {
        for (int i = 0; i < tags.size(); i++) {
            TagObservation tag = tags.get(i);
            GoalTarget.Tag known = targets.get(i).tags.get(tag.id);
            if (Math.abs(mount.cameraHeight+tag.levelPosition(mount)[2]-known.height) > .1
                    || checkPlane && Math.abs(tag.planeDegrees(mount)-HiveTagGeometry.RAISED_TAG_PLANE_DEG) > TAG_PLANE_TOLERANCE_DEG)
                return "Tag " + tag.id + " is not in nominal raised geometry";
        }
        return null;
    }
    /**
     * Null when turret yaw and chassis heading stayed steady around the capture and since it.
     * A turning camera converts capture-latency error into a field-pose error.
     */
    public String fusionMotionRejection(double capture) {
        if (history.isEmpty()) return "Missing capture-time turret / heading history";
        Entry e = at(capture), before = at(capture-FUSION_SETTLE_SECONDS), latest = at(history.getLast().pose.time);
        if (e == null || before == null || latest == null) return "Missing capture-time turret / heading history";
        double settle = MAX_FUSION_TURN_RADIANS_PER_SECOND*FUSION_SETTLE_SECONDS+FUSION_ANGLE_NOISE_RADIANS;
        double since = MAX_FUSION_TURN_RADIANS_PER_SECOND*Math.max(0,latest.pose.time-capture)+FUSION_ANGLE_NOISE_RADIANS;
        if (Math.abs(RobotMath.wrap(e.turretYaw-before.turretYaw)) > settle
                || Math.abs(RobotMath.wrap(latest.turretYaw-e.turretYaw)) > since)
            return "Turret moving near camera capture; waiting for it to settle";
        if (Math.abs(RobotMath.wrap(e.pose.heading-before.pose.heading)) > settle
                || Math.abs(RobotMath.wrap(latest.pose.heading-e.pose.heading)) > since)
            return "Chassis turning near camera capture; waiting for steady view";
        return null;
    }
    /** Field yaw of a raised tag's +X edge: far-cell tags run along +X, audience-cell tags along -X. */
    static double tagEdgeFieldYaw(GoalTarget target) {
        return target.side == TurretSubsystem.HiveSide.AUDIENCE ? Math.PI : 0;
    }
    /** Robot heading implied by one tag's horizontal +X edge at the capture-time turret angle. */
    static double tagHeading(TagObservation tag, GoalTarget target, Entry e, ShooterMount mount) {
        double[] edge = tag.rotate(1,0,0);
        edge = TagObservation.level(edge[0],edge[1],edge[2],mount.cameraPitch);
        return RobotMath.wrap(tagEdgeFieldYaw(target)-Math.atan2(edge[1],edge[0])
                -mount.zeroYaw-e.turretYaw-mount.cameraYaw);
    }
    /** Robot position implied by one tag for a given robot heading. */
    static Vec tagPosition(TagObservation tag, GoalTarget.Tag known, double heading, Entry e, ShooterMount mount) {
        double[] level = tag.levelPosition(mount);
        MotionSample origin = new MotionSample(e.pose.time,0,0,heading,0,0,0,0,.02,true,true,"Vision origin");
        Vec ray = new Vec(level[0],level[1]).rotate(heading+mount.zeroYaw+e.turretYaw+mount.cameraYaw);
        return new Vec(known.x,known.y).minus(ray).minus(mount.cameraPosition(origin,e.turretYaw));
    }
    /** Averages heading, then position, over the frame's tags. out = {heading, x, y, uncertainty}. */
    private static String fuseFrame(List<TagObservation> tags, List<GoalTarget> targets, Entry e, ShooterMount mount, double[] out) {
        double sin = 0, cos = 0;
        double[] headings = new double[tags.size()];
        for (int i = 0; i < tags.size(); i++) {
            headings[i] = tagHeading(tags.get(i),targets.get(i),e,mount);
            sin += Math.sin(headings[i]); cos += Math.cos(headings[i]);
        }
        double heading = Math.atan2(sin,cos);
        for (int i = 0; i < tags.size(); i++) if (Math.abs(RobotMath.wrap(headings[i]-heading)) > FRAME_HEADING_AGREEMENT_RAD)
            return "Tags disagree on heading in one frame";
        Vec[] positions = new Vec[tags.size()];
        double x = 0, y = 0, uncertainty = Double.POSITIVE_INFINITY;
        for (int i = 0; i < tags.size(); i++) {
            TagObservation tag = tags.get(i);
            positions[i] = tagPosition(tag,targets.get(i).tags.get(tag.id),heading,e,mount);
            x += positions[i].x; y += positions[i].y; uncertainty = Math.min(uncertainty,tag.uncertaintyMeters());
        }
        x /= tags.size(); y /= tags.size();
        for (Vec position : positions) if (Math.hypot(position.x-x,position.y-y) > FRAME_POSITION_AGREEMENT_METERS)
            return "Tags disagree on position in one frame";
        out[0] = heading; out[1] = x; out[2] = y; out[3] = uncertainty/Math.sqrt(tags.size());
        return null;
    }
    private static String ids(List<TagObservation> tags) {
        StringBuilder b = new StringBuilder();
        for (TagObservation tag : tags) b.append(b.length() == 0 ? "" : ", ").append(tag.id);
        return b.toString();
    }
    private boolean rejectAcquisition(String message) { candidateFrames = 0; return reject(message); }
    /** Known tag positions are state-specific. Never consume botpose as though it were camera pose. */
    public boolean observe(double now, double frameId, double captureTime, int tagId, double tx, double ty,
                           GoalTarget target, ShooterMount mount) {
        if (!RobotMath.finite(now, frameId, captureTime, tx, ty) || frameId <= lastFrame) return reject("Duplicate / out-of-order vision frame");
        lastFrame = frameId;
        if (now < captureTime || now - captureTime > MAX_OBSERVATION_AGE_SECONDS) return reject("Stale vision frame");
        if (!target.scoreable() || !mount.measured) return reject("Unmeasured or incompatible hive/mount geometry");
        GoalTarget.Tag tag = target.tags.get(tagId);
        if (tag == null) return reject("Tag outside selected opening/state map");
        Entry e = at(captureTime);
        if (e == null || !e.pose.fieldReferenced) return reject("Missing capture history / starting field reference");
        double yaw = e.pose.heading + mount.zeroYaw + e.turretYaw + mount.cameraYaw;
        Vec ray = mount.cameraToTag(tag, tx, ty, yaw);
        if (ray == null) return reject("Invalid tag-height range");
        Vec observedCamera = new Vec(tag.x, tag.y).minus(ray);
        Vec delta = observedCamera.minus(mount.cameraPosition(e.pose, e.turretYaw));
        if (delta.norm() > MAX_JUMP_METERS) return reject("Implausible vision position jump");
        // Correct history in a shared transform, never by differentiating a pose reset.
        correctionX += delta.x * CORRECTION_GAIN; correctionY += delta.y * CORRECTION_GAIN;
        acceptedUncertainty = Math.hypot(.05,delta.norm()*.65);
        lastObservation = captureTime; reason = "Accepted tag " + tagId; return true;
    }
    /** Single-tag form kept for the diagnostic path; see the multi-tag overload. */
    public boolean observe3d(double now, double frame, double capture, TagObservation tag, GoalTarget target, ShooterMount mount) {
        return observe3d(now,frame,capture,tag == null ? null : Collections.singletonList(tag),Collections.singletonList(target),mount);
    }
    /**
     * Multi-tag 3D fusion after a field reference exists. Moves position and heading a fixed fraction
     * toward the frame's averaged estimate (heading rotates about the capture-time position, so the
     * robot does not jump). Does not infer range from a configured tag height.
     */
    public boolean observe3d(double now, double frame, double capture, List<TagObservation> tags,
                             List<GoalTarget> targets, ShooterMount mount) {
        if (tags == null || tags.isEmpty() || tags.contains(null)) return reject("Missing metric observation pose");
        for (TagObservation tag : tags) {
            String rejection = tag.rejectionReason();
            if (rejection != null) return reject("Tag " + tag.id + ": " + rejection);
        }
        if (!RobotMath.finite(now,frame,capture) || frame <= lastFrame) return reject("Invalid / duplicate 3D observation");
        lastFrame = frame;
        if (now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS) return reject("Stale 3D observation");
        Entry e = at(capture);
        if (e == null || !e.pose.fieldReferenced || mapRejection(tags,targets,mount) != null) return reject("Missing field map / capture reference");
        if (geometryRejection(tags,targets,mount,false) != null) return reject("Implausible 3D field correction");
        String motion = fusionMotionRejection(capture);
        if (motion != null) return reject(motion);
        double[] fix = new double[4];
        String disagreement = fuseFrame(tags,targets,e,mount,fix);
        if (disagreement != null) return reject(disagreement);
        double headingError = RobotMath.wrap(fix[0]-e.pose.heading);
        if (Math.abs(headingError) > MAX_HEADING_CORRECTION_RAD)
            return reject(String.format(java.util.Locale.US,"Vision heading differs by %.1f deg; correction rejected",Math.toDegrees(headingError)));
        Vec capturePosition = new Vec(e.pose.x,e.pose.y);
        Vec delta = new Vec(fix[1],fix[2]).minus(capturePosition);
        if (delta.norm() > MAX_JUMP_METERS) return reject("Implausible 3D field correction");
        double turn = headingError*CORRECTION_GAIN;
        Vec rotated = new Vec(correctionX,correctionY).minus(capturePosition).rotate(turn).plus(capturePosition);
        correctionYaw = RobotMath.wrap(correctionYaw+turn);
        correctionX = rotated.x+delta.x*CORRECTION_GAIN; correctionY = rotated.y+delta.y*CORRECTION_GAIN;
        acceptRelative(capture,Math.hypot(fix[3],delta.norm()*.65)); reason = "Accepted metric tags " + ids(tags);
        return true;
    }
    /**
     * Drift-only check used by TeleOp and Auto Aim once a field reference exists. Accepted frames refresh
     * observation age; the pose changes only for consistent disagreement beyond the deadband.
     */
    public boolean checkDrift3d(double now, double frame, double capture, List<TagObservation> tags,
                                List<GoalTarget> targets, ShooterMount mount) {
        if (tags == null || tags.isEmpty() || tags.contains(null)) return reject("Missing metric observation pose");
        for (TagObservation tag : tags) {
            String rejection = tag.rejectionReason();
            if (rejection != null) return reject("Tag " + tag.id + ": " + rejection);
        }
        if (!RobotMath.finite(now,frame,capture) || frame <= lastFrame) return reject("Invalid / duplicate 3D observation");
        lastFrame = frame;
        if (now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS) return reject("Stale 3D observation");
        if (blendRemaining > 0) return reject("Applying odometry drift correction");
        Entry e = at(capture);
        if (e == null || !e.pose.fieldReferenced || mapRejection(tags,targets,mount) != null) return reject("Missing field map / capture reference");
        if (geometryRejection(tags,targets,mount,false) != null) return reject("Implausible 3D field correction");
        String motion = fusionMotionRejection(capture);
        if (motion != null) return reject(motion);
        double[] fix = new double[4];
        String disagreement = fuseFrame(tags,targets,e,mount,fix);
        if (disagreement != null) return reject(disagreement);
        double headingError = RobotMath.wrap(fix[0]-e.pose.heading);
        double dx = fix[1]-e.pose.x, dy = fix[2]-e.pose.y;
        if (Math.hypot(dx,dy) > MAX_JUMP_METERS || Math.abs(headingError) > MAX_HEADING_CORRECTION_RAD)
            return largeDisagreement(now,capture,tags.size(),fix,e.pose);
        largeFrames.clear(); // This frame agrees with odometry: no evidence that the reference is wrong.
        while (!driftFrames.isEmpty() && (capture-driftFrames.peekFirst()[0] > DRIFT_WINDOW_SECONDS || driftFrames.size() >= DRIFT_FRAMES))
            driftFrames.removeFirst();
        driftFrames.addLast(new double[]{capture,headingError,dx,dy,e.pose.x,e.pose.y});
        acceptRelative(capture,fix[3]);
        if (driftFrames.size() < DRIFT_FRAMES) {
            reason = "Checking odometry against camera " + driftFrames.size() + "/" + DRIFT_FRAMES;
            return true;
        }
        double sin = 0, cos = 0, meanX = 0, meanY = 0, pivotX = 0, pivotY = 0;
        for (double[] d : driftFrames) {
            sin += Math.sin(d[1]); cos += Math.cos(d[1]); meanX += d[2]; meanY += d[3]; pivotX += d[4]; pivotY += d[5];
        }
        int n = driftFrames.size();
        double meanYaw = Math.atan2(sin,cos); meanX /= n; meanY /= n; pivotX /= n; pivotY /= n;
        for (double[] d : driftFrames) if (Math.abs(RobotMath.wrap(d[1]-meanYaw)) > DRIFT_FRAME_AGREEMENT_RAD
                || Math.hypot(d[2]-meanX,d[3]-meanY) > DRIFT_FRAME_AGREEMENT_METERS) {
            reason = "Camera frames inconsistent; odometry kept";
            return true;
        }
        String delta = String.format(java.util.Locale.US,"%.1f cm, %.1f deg",100*Math.hypot(meanX,meanY),Math.toDegrees(meanYaw));
        if (Math.hypot(meanX,meanY) <= DRIFT_POSITION_DEADBAND_METERS && Math.abs(meanYaw) <= DRIFT_HEADING_DEADBAND_RAD) {
            reason = "Odometry agrees with camera (" + delta + ")";
            return true;
        }
        blendYaw = meanYaw; blendX = meanX; blendY = meanY; blendPivotX = pivotX; blendPivotY = pivotY;
        blendRemaining = DRIFT_BLEND_SECONDS; blendTime = history.getLast().pose.time;
        driftFrames.clear();
        reason = "Odometry drift corrected (" + delta + ")";
        return true;
    }
    /**
     * One settled frame whose camera pose disagrees with odometry beyond the drift limits. Collects consistent
     * multi-tag evidence and, once confirmed, re-localizes through the same blended rigid correction as drift.
     */
    private boolean largeDisagreement(double now, double capture, int tagCount, double[] fix, MotionSample odometry) {
        double headingError = RobotMath.wrap(fix[0]-odometry.heading);
        String size = String.format(java.util.Locale.US,"%.0f cm / %.1f deg",
                100*Math.hypot(fix[1]-odometry.x,fix[2]-odometry.y),Math.toDegrees(headingError));
        String disagreement = "Camera disagrees with odometry by " + size;
        largeSeenAt = now;
        if (tagCount < RELOCALIZE_MIN_TAGS) {
            largeFrames.clear(); largeStatus = size + " | needs " + RELOCALIZE_MIN_TAGS + "+ tags in view";
            return reject(disagreement + "; ignored (re-localizing needs " + RELOCALIZE_MIN_TAGS + "+ tags per frame)");
        }
        if (Math.abs(fix[1]) > TeleOpFieldReference.FIELD_HALF_WIDTH_METERS || Math.abs(fix[2]) > TeleOpFieldReference.FIELD_HALF_WIDTH_METERS) {
            largeFrames.clear(); largeStatus = size + " | camera pose off the field";
            return reject(disagreement + "; ignored (camera pose outside field)");
        }
        while (!largeFrames.isEmpty() && (capture-largeFrames.peekFirst()[0] > RELOCALIZE_WINDOW_SECONDS || largeFrames.size() >= 256))
            largeFrames.removeFirst();
        largeFrames.addLast(new double[]{capture,fix[0],fix[1],fix[2],odometry.heading,odometry.x,odometry.y});
        double span = capture-largeFrames.peekFirst()[0];
        if (largeFrames.size() < RELOCALIZE_FRAMES || span < RELOCALIZE_MIN_SPAN_SECONDS) {
            largeStatus = String.format(java.util.Locale.US,"%s | confirming %d/%d frames, %.1f/%.1f s",size,
                    Math.min(largeFrames.size(),RELOCALIZE_FRAMES),RELOCALIZE_FRAMES,span,RELOCALIZE_MIN_SPAN_SECONDS);
            return reject(disagreement + "; confirming " + Math.min(largeFrames.size(),RELOCALIZE_FRAMES) + "/" + RELOCALIZE_FRAMES + " frames");
        }
        // The rigid correction rotates about the mean odometry position, then translates: frame i is matched when
        // move_i = camera_i - pivot - R(turn)(odometry_i - pivot). Every frame must imply the same turn and move.
        double sin = 0, cos = 0, pivotX = 0, pivotY = 0;
        for (double[] f : largeFrames) {
            double turn = RobotMath.wrap(f[1]-f[4]);
            sin += Math.sin(turn); cos += Math.cos(turn); pivotX += f[5]; pivotY += f[6];
        }
        int n = largeFrames.size();
        double turn = Math.atan2(sin,cos); pivotX /= n; pivotY /= n;
        Vec pivot = new Vec(pivotX,pivotY);
        Vec[] moves = new Vec[n];
        double moveX = 0, moveY = 0;
        int i = 0;
        for (double[] f : largeFrames) {
            moves[i] = new Vec(f[2],f[3]).minus(pivot).minus(new Vec(f[5],f[6]).minus(pivot).rotate(turn));
            moveX += moves[i].x; moveY += moves[i].y; i++;
        }
        moveX /= n; moveY /= n;
        i = 0;
        for (double[] f : largeFrames) {
            if (Math.abs(RobotMath.wrap(f[1]-f[4]-turn)) > DRIFT_FRAME_AGREEMENT_RAD
                    || Math.hypot(moves[i].x-moveX,moves[i].y-moveY) > DRIFT_FRAME_AGREEMENT_METERS) {
                largeFrames.clear(); largeStatus = size + " | camera frames inconsistent";
                return reject(disagreement + "; camera frames inconsistent, odometry kept");
            }
            i++;
        }
        blendYaw = turn; blendX = moveX; blendY = moveY; blendPivotX = pivotX; blendPivotY = pivotY;
        blendRemaining = DRIFT_BLEND_SECONDS; blendTime = history.getLast().pose.time;
        driftFrames.clear(); largeFrames.clear();
        relocalizedAt = now;
        relocalizedSummary = String.format(java.util.Locale.US,"moved %.0f cm, turned %.1f deg",
                100*Math.hypot(moveX,moveY),Math.toDegrees(turn));
        acceptRelative(capture,fix[3]);
        reason = "Odometry re-localized from camera after " + n + " consistent frames (" + relocalizedSummary + ")";
        return true;
    }
    /** Driver view: a recent re-localization or large camera disagreement; null when there is nothing to report. */
    public String largeCorrectionStatus(double now) {
        if (now >= relocalizedAt && now-relocalizedAt <= RELOCALIZED_DISPLAY_SECONDS)
            return "RE-LOCALIZED from camera (" + relocalizedSummary + ")";
        if (now >= largeSeenAt && now-largeSeenAt <= DISAGREEMENT_DISPLAY_SECONDS)
            return "CAMERA DISAGREES " + largeStatus;
        return null;
    }
    /** Spreads a drift correction over several loops: rotate about the drift pivot, then translate. */
    private void applyBlend(double time) {
        if (blendRemaining <= 0 || !(time > blendTime)) return;
        double dt = time-blendTime, fraction = Math.min(1,dt/blendRemaining);
        double turn = blendYaw*fraction, moveX = blendX*fraction, moveY = blendY*fraction;
        Vec pivot = new Vec(blendPivotX,blendPivotY);
        Vec rotated = new Vec(correctionX,correctionY).minus(pivot).rotate(turn).plus(pivot);
        correctionYaw = RobotMath.wrap(correctionYaw+turn);
        correctionX = rotated.x+moveX; correctionY = rotated.y+moveY;
        blendPivotX += moveX; blendPivotY += moveY;
        blendYaw -= turn; blendX -= moveX; blendY -= moveY;
        blendRemaining -= dt; blendTime = time;
        if (fraction >= 1) blendRemaining = 0;
    }
    /** Target-relative measurement constrains relative aiming uncertainty, never creates a field reference. */
    public void acceptRelative(double capture, double uncertainty) {
        if (!RobotMath.finite(capture,uncertainty) || uncertainty <= 0) return;
        acceptedUncertainty = Math.max(.025,uncertainty); lastObservation = capture;
    }
    /** Camera error from capture pose to the current chassis frame, with stale/missing history rejected. */
    public double delayedYaw(double now, double capture, double txRad, ShooterMount mount) {
        return delayedYaw(now,capture,txRad,0,mount);
    }
    public double delayedYaw(double now, double capture, double txRad, double tyRad, ShooterMount mount) {
        return delayedYaw(now,capture,txRad,tyRad,mount,0);
    }
    /** leadSeconds: the turret command leads the current heading by the actuation delay. */
    public double delayedYaw(double now, double capture, double txRad, double tyRad, ShooterMount mount, double leadSeconds) {
        Entry e = at(capture); MotionSample current = current(now);
        if (e == null || current == null || !current.fresh(now,.1) || now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS
                || !RobotMath.finite(txRad,tyRad) || Math.abs(txRad)>=Math.PI/2 || Math.abs(tyRad)>=Math.PI/2) return Double.NaN;
        // Without metric range, constrain the uncorrected parallax from camera translation.
        if (mount.cameraPosition(current,history.getLast().turretYaw).minus(mount.cameraPosition(e.pose,e.turretYaw)).norm() > .03) return Double.NaN;
        double forward=Math.cos(mount.cameraPitch)-Math.tan(tyRad)*Math.sin(mount.cameraPitch);
        if (forward <= 0) return Double.NaN;
        double bearing=Math.atan2(-Math.tan(txRad),forward);
        return RobotMath.wrap(e.pose.heading+e.turretYaw+mount.cameraYaw+bearing-current.led(leadSeconds).heading);
    }
    public double delayedYaw3d(double now, double capture, TagObservation tag, ShooterMount mount) {
        return delayedYaw3d(now,capture,tag,mount,0);
    }
    public double delayedYaw3d(double now, double capture, TagObservation tag, ShooterMount mount, double leadSeconds) {
        Entry e=at(capture); MotionSample measured=current(now);
        MotionSample current=measured == null ? null : measured.led(leadSeconds);
        if (tag == null || !tag.usable() || e == null || current == null || !current.fresh(now,.1)
                || now < capture || now-capture > MAX_OBSERVATION_AGE_SECONDS) return Double.NaN;
        double[] level=tag.levelPosition(mount);
        Vec target=mount.cameraPosition(e.pose,e.turretYaw).plus(new Vec(level[0],level[1])
                .rotate(e.pose.heading+mount.zeroYaw+e.turretYaw+mount.cameraYaw));
        Vec currentPivot=new Vec(current.x,current.y).plus(mount.pivot.rotate(current.heading));
        Vec direction=target.minus(currentPivot);
        if (direction.norm()<.1) return Double.NaN;
        return RobotMath.wrap(direction.angle()-current.heading-mount.zeroYaw);
    }
    private boolean reject(String message) { reason = message; return false; }
    public double observationAge(double now) { return now - lastObservation; }
    public boolean targetFresh(double now) { double age = observationAge(now); return age >= 0 && age <= MAX_TARGET_AGE_SECONDS; }
    public String reason() { return reason; }
    /** SDK 12 hub timestamp is currentTimeMillis at parsing start. Age already includes JSON parse time. */
    public static double captureTime(double monotonicNow, long wallNowMs, long hubReceiptMs,
                                     double captureLatencyMs, double pipelineLatencyMs, double transportSeconds) {
        return captureTimeFromAge(monotonicNow, wallNowMs - hubReceiptMs,
                captureLatencyMs, pipelineLatencyMs, transportSeconds);
    }
    /** Use SDK receipt age directly; it avoids mixing device and application wall-clock epochs. */
    public static double captureTimeFromAge(double monotonicNow, double receiptAgeMs,
                                            double captureLatencyMs, double pipelineLatencyMs, double transportSeconds) {
        double age = receiptAgeMs * .001 + (captureLatencyMs + pipelineLatencyMs) * .001 + transportSeconds;
        if (!RobotMath.finite(age, captureLatencyMs, pipelineLatencyMs, transportSeconds) || age < 0
                || receiptAgeMs < 0 || captureLatencyMs < 0 || pipelineLatencyMs < 0 || transportSeconds < 0) return Double.NaN;
        return monotonicNow - age;
    }
}
