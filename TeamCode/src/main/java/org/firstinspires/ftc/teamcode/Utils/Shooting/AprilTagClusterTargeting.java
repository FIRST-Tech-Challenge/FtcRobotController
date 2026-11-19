package org.firstinspires.ftc.teamcode.Utils.Shooting;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Utils.Positioning.DistanceToHive;
import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Finds the center of the BioBuzz Hive OPENING using every April Tag it can see on a hive,
 * instead of just pointing at one tag.
 *
 * <p>This class is standalone: it only reads the Limelight and gives back numbers.
 * It does not move any motors. You can feed {@link #getTx()} into whatever turret code you want
 * (it has the same sign / units as Limelight tx, so it can drop in where tx is used now).
 *
 * <h3>How it works</h3>
 * <ol>
 *   <li>Grab every fiducial the Limelight sees this frame.</li>
 *   <li>Group them into clusters (one cluster per hive). Tags you registered with
 *       {@link #addHive(String)} are grouped by hive; unregistered tags are grouped by how
 *       close they are to each other.</li>
 *   <li>Pick the best cluster (most tags, then biggest total area).</li>
 *   <li>Work out where the opening is:
 *     <ul>
 *       <li><b>2+ known tags (3D):</b> fit the hive's tag layout to where the tags actually are
 *           (best-fit rotation + translation, weighted by tag size). The opening is wherever the
 *           layout's (0,0) lands. Works even when only part of the cluster is visible.</li>
 *       <li><b>1 known tag (3D):</b> use that tag's position + its yaw to step from the tag to the
 *           opening.</li>
 *       <li><b>Unknown tags, or no 3D data:</b> area-weighted average of the tags (center of the cluster).</li>
 *     </ul>
 *   </li>
 *   <li>Optionally smooth the result and hold it for a short time if the tags drop out.</li>
 * </ol>
 *
 * <h3>Setup</h3>
 * The 4 BioBuzz hive clusters (tags 30-45) from the FTC SDK are registered automatically.
 * <pre>
 * AprilTagClusterTargeting hive = new AprilTagClusterTargeting();
 * hive.setTiltServo(lla);                                     // camera tilt
 * hive.allowedHives.add(AprilTagClusterTargeting.RED_SCORING); // optional: only aim at this hive
 *
 * // every loop
 * if (hive.update(limelight)) {
 *     double tx = hive.getTx();   // degrees, + = opening is to the right
 * }
 * </pre>
 *
 * The Limelight pipeline needs "Full 3D" turned on and the tag size set to 3.25 in (82.55 mm)
 * for the layout fitting to work. Without 3D the class still works but falls back to averaging the tags.
 */
public class AprilTagClusterTargeting {

    // ------------------------------------------------------------------------------------------
    // Tunables
    // ------------------------------------------------------------------------------------------

    /**
     * Camera tilt in degrees (+ = tilted up). Used to level out the 3D tag positions.
     * If a tilt servo is set with {@link #setTiltServo}, this is recalculated every update.
     */
    public double cameraPitchDeg = 0;

    // ---- Tilting camera servo (LLA) ----
    // Servo has no position feedback in the SDK: getPosition() is the last COMMANDED position.
    // That position is turned into degrees by servoPositionToPitchDeg() below.
    private Servo tiltServo = null;

    /** Time (ms) we assume the servo needs to reach a new position. Results while moving are flagged. */
    public long tiltSettleMs = 200;
    private double lastTiltCommand = Double.NaN;
    private long tiltChangedMs = 0;

    /** Camera position relative to the turret pivot, inches (+right, +forward). 0 if the camera is on the pivot. */
    public double cameraOffsetRightIn = 0;
    public double cameraOffsetForwardIn = 0;

    /** Unregistered tags closer than this (inches, 3D) or this (degrees, no 3D) count as the same cluster. */
    public double clusterRadiusIn = 30;
    public double clusterRadiusDeg = 12;

    /** Use the tag's yaw when only one known tag is visible. Flip the sign if single-tag aim goes the wrong way. */
    public boolean useSingleTagYaw = true;
    public double singleTagYawSign = 1;

    /** 0..1, how much of the new reading to use each loop. 1 = no smoothing. */
    public double smoothing = 0.6;

    /** How long (ms) to keep reporting the last result after the tags disappear. */
    public long holdTimeMs = 150;

    /** If not empty, only these hive names are aimed at (e.g. {@link #RED_SCORING}). Empty = any hive. */
    public final Set<String> allowedHives = new HashSet<>();

    /** Pipeline to force the Limelight onto. Set to -1 to leave it alone. */
    public int limelightPipeline;

    // ------------------------------------------------------------------------------------------
    // Hive layouts
    // ------------------------------------------------------------------------------------------

    /** A hive's tag layout. Positions are inches from the opening center: x right, y toward viewer, z up. */
    public static class HiveLayout {
        public final String name;
        final Map<Integer, double[]> tags = new LinkedHashMap<>();

        HiveLayout(String name) {
            this.name = name;
        }

        public HiveLayout tag(int id, double rightIn, double towardViewerIn, double upIn) {
            tags.put(id, new double[]{rightIn, towardViewerIn, upIn});
            return this;
        }
    }

    private final List<HiveLayout> hives = new ArrayList<>();

    public HiveLayout addHive(String name) {
        HiveLayout layout = new HiveLayout(name);
        hives.add(layout);
        return layout;
    }

    // ---- BioBuzz layout, copied from the FTC SDK 12.0 ----
    // Source: org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getBioBuzzTagLibrary()
    // The SDK keeps the member positions protected, so the numbers are copied here instead of read at runtime.
    // SDK cluster frame (from AprilTagProcessorImpl.doClusterSolve): x = right, y = up the tag face,
    // z = out of the tag face toward the viewer; the cluster origin (0,0,0) is the opening center.
    public static final String RED_SCORING   = "RED SCORING";   // tags 30-33
    public static final String RED_AUDIENCE  = "RED AUDIENCE";  // tags 34-37
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE"; // tags 38-41
    public static final String BLUE_SCORING  = "BLUE SCORING";  // tags 42-45

    public static final double BIOBUZZ_TAG_SIZE_IN = 3.25;
    private static final double BIOBUZZ_TAG_UP_IN = 7.1874;      // SDK offsetY
    private static final double BIOBUZZ_TAG_TOWARD_IN = -5.622;  // SDK offsetZ
    private static final double[] BIOBUZZ_TAG_RIGHT_IN = {-6.50, -2.75, 2.75, 6.50}; // L1, L2, R1, R2

    /**
     * Slope of the hive's tag face: 30 deg up from horizontal (= 60 deg down from vertical).
     * The SDK offsets are measured along / out of this sloped face, so they get rotated into level
     * (toward viewer, up) coordinates:
     *   up-the-face  direction = (toward: -cos a, up: sin a)   (goes up and away from you)
     *   out-of-face  direction = (toward:  sin a, up: cos a)   (points at you and up)
     */
    public static final double BIOBUZZ_FACE_ANGLE_FROM_HORIZONTAL_DEG = 30;

    /** Registers all 4 BioBuzz hive clusters exactly as the SDK defines them. Done automatically by the constructors. */
    public void addBioBuzzHives() {
        addBioBuzzHive(RED_SCORING, 30);
        addBioBuzzHive(RED_AUDIENCE, 34);
        addBioBuzzHive(BLUE_AUDIENCE, 38);
        addBioBuzzHive(BLUE_SCORING, 42);
    }

    private void addBioBuzzHive(String name, int firstId) {
        double a = Math.toRadians(BIOBUZZ_FACE_ANGLE_FROM_HORIZONTAL_DEG);
        double alongFace = BIOBUZZ_TAG_UP_IN;      // SDK y
        double outOfFace = BIOBUZZ_TAG_TOWARD_IN;  // SDK z
        // At 30 deg this comes out to about 9.04 in behind and 1.28 in below the opening
        double toward = alongFace * -Math.cos(a) + outOfFace * Math.sin(a);
        double up     = alongFace *  Math.sin(a) + outOfFace * Math.cos(a);

        HiveLayout h = addHive(name);
        for (int i = 0; i < BIOBUZZ_TAG_RIGHT_IN.length; i++) {
            h.tag(firstId + i, BIOBUZZ_TAG_RIGHT_IN[i], toward, up);
        }
    }

    // ------------------------------------------------------------------------------------------
    // Results
    // ------------------------------------------------------------------------------------------

    public enum Method { NONE, LAYOUT_FIT, SINGLE_TAG, CLUSTER_AVERAGE, HELD }

    private boolean hasTarget = false;
    private double tx = 0;            // degrees, + = right (same as Limelight tx)
    private double ty = 0;            // degrees above level (or Limelight ty when no 3D)
    private double distanceIn = -1;   // horizontal distance turret pivot -> opening, -1 if unknown
    private double heightIn = 0;      // opening height relative to the camera, inches
    private int tagsUsed = 0;
    private String hiveName = "";
    private Method method = Method.NONE;
    private List<Integer> tagIds = new ArrayList<>();

    private boolean hadValue = false;
    private long lastSeenMs = 0;

    public AprilTagClusterTargeting() {
        // Same pipelines AprilTagsTurretTarget uses: 4 red, 3 blue
        this(LinearOpMode2026.isCurrentAllienceRed ? 4 : 3);
    }

    public AprilTagClusterTargeting(int limelightPipeline) {
        this.limelightPipeline = limelightPipeline;
        addBioBuzzHives();
    }

    // ------------------------------------------------------------------------------------------
    // Main update
    // ------------------------------------------------------------------------------------------

    /** Internal per-tag data. Leveled camera frame, inches: x right, f forward, u up. */
    private static class Tag {
        int id;
        double txDeg, tyDeg, area, weight;
        boolean has3d;
        double x, f, u;
        double yawDeg;
        HiveLayout hive;
        double[] layoutPos;
    }

    /**
     * Read the Limelight and recompute the opening position. Call once per loop.
     *
     * @return true if there is a target (fresh or held)
     */
    public boolean update(Limelight3A limelight) {
        updateTiltFromServo();

        if (limelight == null) return finishNoTags();

        if (limelightPipeline >= 0 && limelight.getStatus().getPipelineIndex() != limelightPipeline) {
            limelight.pipelineSwitch(limelightPipeline);
        }

        return update(limelight.getLatestResult());
    }

    /** Same as {@link #update(Limelight3A)} but with a result you already grabbed. */
    public boolean update(LLResult result) {
        if (result == null || !result.isValid()) return finishNoTags();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return finishNoTags();

        List<Tag> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            tags.add(toTag(fr));
        }

        List<List<Tag>> clusters = buildClusters(tags);
        List<Tag> best = pickBestCluster(clusters);
        if (best == null || best.isEmpty()) return finishNoTags();

        double[] aim = solve(best); // {tx, ty, distance, height, methodOrdinal}
        if (aim == null) return finishNoTags();

        applyResult(aim, best);
        return true;
    }

    // ------------------------------------------------------------------------------------------
    // Camera tilt servo
    // ------------------------------------------------------------------------------------------

    /**
     * Give this class the camera tilt servo (LLA) so it reads the camera angle by itself every update.
     * <pre>hive.setTiltServo(lla);</pre>
     */
    public void setTiltServo(Servo servo) {
        tiltServo = servo;
        updateTiltFromServo();
    }

    /**
     * Servo position (0..1) -> camera pitch in degrees (+ = tilted up from level).
     *
     * TODO: write the conversion for our LLA servo. Until this is filled in the camera is treated as level.
     */
    public double servoPositionToPitchDeg(double servoPos) {
        if (distanceToHive == null) return 0; // call setDistanceToHive(...) first
        double cameraAngle = (servoPos * 300) - 150;
        return cameraAngle;
    }

    /** Odometry-based hive distance used by servoPositionToPitchDeg. */
    private DistanceToHive distanceToHive = null;

    public void setDistanceToHive(DistanceToHive distanceToHive) {
        this.distanceToHive = distanceToHive;
    }

    /** Use this instead of setTiltServo if you'd rather pass the servo position in yourself each loop. */
    public void setCameraPitchFromServoPosition(double servoPos) {
        noteTiltCommand(servoPos);
        cameraPitchDeg = servoPositionToPitchDeg(servoPos);
    }

    private void updateTiltFromServo() {
        if (tiltServo == null) return;
        setCameraPitchFromServoPosition(tiltServo.getPosition());
    }

    private void noteTiltCommand(double servoPos) {
        if (Double.isNaN(lastTiltCommand) || Math.abs(servoPos - lastTiltCommand) > 1e-4) {
            if (!Double.isNaN(lastTiltCommand)) tiltChangedMs = System.currentTimeMillis();
            lastTiltCommand = servoPos;
        }
    }

    /** True for {@link #tiltSettleMs} after the tilt servo was told to move (the angle may not be exact yet). */
    public boolean isTiltSettling() {
        return System.currentTimeMillis() - tiltChangedMs < tiltSettleMs;
    }

    private Tag toTag(LLResultTypes.FiducialResult fr) {
        Tag t = new Tag();
        t.id = fr.getFiducialId();
        t.txDeg = fr.getTargetXDegrees();
        t.tyDeg = fr.getTargetYDegrees();
        t.area = fr.getTargetArea();
        t.weight = Math.max(t.area, 1e-4);

        for (HiveLayout h : hives) {
            double[] p = h.tags.get(t.id);
            if (p != null) {
                t.hive = h;
                t.layoutPos = p;
                break;
            }
        }

        Pose3D pose = fr.getTargetPoseCameraSpace();
        if (pose != null && pose.getPosition() != null) {
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            // Limelight camera space: +x right, +y down, +z forward
            if (Math.abs(p.x) + Math.abs(p.y) + Math.abs(p.z) > 1e-6) {
                t.has3d = true;
                double pitch = Math.toRadians(cameraPitchDeg);
                double camY = p.y, camZ = p.z;
                t.x = p.x + cameraOffsetRightIn;
                t.f = camZ * Math.cos(pitch) - camY * Math.sin(pitch) + cameraOffsetForwardIn;
                t.u = camZ * Math.sin(pitch) - camY * Math.cos(pitch);
                t.yawDeg = pose.getOrientation() != null ? pose.getOrientation().getYaw(AngleUnit.DEGREES) : 0;
            }
        }
        return t;
    }

    // ------------------------------------------------------------------------------------------
    // Clustering
    // ------------------------------------------------------------------------------------------

    private List<List<Tag>> buildClusters(List<Tag> tags) {
        List<List<Tag>> clusters = new ArrayList<>();

        // Known tags: one cluster per hive layout
        for (HiveLayout h : hives) {
            List<Tag> c = new ArrayList<>();
            for (Tag t : tags) if (t.hive == h) c.add(t);
            if (!c.isEmpty()) clusters.add(c);
        }

        // Unknown tags: group by distance (single linkage)
        List<Tag> unknown = new ArrayList<>();
        for (Tag t : tags) if (t.hive == null) unknown.add(t);

        boolean[] used = new boolean[unknown.size()];
        for (int i = 0; i < unknown.size(); i++) {
            if (used[i]) continue;
            List<Tag> c = new ArrayList<>();
            c.add(unknown.get(i));
            used[i] = true;
            for (int k = 0; k < c.size(); k++) {
                for (int j = 0; j < unknown.size(); j++) {
                    if (!used[j] && close(c.get(k), unknown.get(j))) {
                        used[j] = true;
                        c.add(unknown.get(j));
                    }
                }
            }
            clusters.add(c);
        }
        return clusters;
    }

    private boolean close(Tag a, Tag b) {
        if (a.has3d && b.has3d) {
            double dx = a.x - b.x, df = a.f - b.f, du = a.u - b.u;
            return Math.sqrt(dx * dx + df * df + du * du) <= clusterRadiusIn;
        }
        double dx = a.txDeg - b.txDeg, dy = a.tyDeg - b.tyDeg;
        return Math.sqrt(dx * dx + dy * dy) <= clusterRadiusDeg;
    }

    private List<Tag> pickBestCluster(List<List<Tag>> clusters) {
        List<Tag> best = null;
        double bestArea = -1;
        boolean bestKnown = false;

        for (List<Tag> c : clusters) {
            boolean known = c.get(0).hive != null;
            if (!allowedHives.isEmpty() && (!known || !allowedHives.contains(c.get(0).hive.name))) continue;

            double area = 0;
            for (Tag t : c) area += t.area;

            boolean better;
            if (best == null) better = true;
            else if (known != bestKnown) better = known;            // registered hives win over random tags
            else if (c.size() != best.size()) better = c.size() > best.size();
            else better = area > bestArea;

            if (better) {
                best = c;
                bestArea = area;
                bestKnown = known;
            }
        }
        return best;
    }

    // ------------------------------------------------------------------------------------------
    // Solving for the opening
    // ------------------------------------------------------------------------------------------

    private double[] solve(List<Tag> cluster) {
        List<Tag> known3d = new ArrayList<>();
        boolean all3d = true;
        for (Tag t : cluster) {
            if (!t.has3d) all3d = false;
            if (t.has3d && t.layoutPos != null) known3d.add(t);
        }

        if (known3d.size() >= 2) {
            double[] r = layoutFit(known3d);
            if (r != null) return r;
        }
        if (known3d.size() == 1) {
            return singleTag(known3d.get(0));
        }
        return clusterAverage(cluster, all3d);
    }

    /**
     * Best-fit 2D rotation + translation (top-down view) from the hive layout to what the camera sees.
     * The opening is the layout origin, so it lands at the fitted translation.
     */
    private double[] layoutFit(List<Tag> tags) {
        // Layout plane coords: (a = right, b = toward viewer)
        // Camera plane coords:  (x = right, w = toward camera = -forward)  -> same handedness
        double sw = 0, ax = 0, bx = 0, cx = 0, cw = 0;
        for (Tag t : tags) {
            sw += t.weight;
            ax += t.weight * t.layoutPos[0];
            bx += t.weight * t.layoutPos[1];
            cx += t.weight * t.x;
            cw += t.weight * -t.f;
        }
        ax /= sw; bx /= sw; cx /= sw; cw /= sw;

        double sDot = 0, sCross = 0;
        for (Tag t : tags) {
            double a = t.layoutPos[0] - ax, b = t.layoutPos[1] - bx;
            double x = t.x - cx, w = -t.f - cw;
            sDot += t.weight * (a * x + b * w);
            sCross += t.weight * (a * w - b * x);
        }

        double theta;
        if (Math.abs(sDot) + Math.abs(sCross) < 1e-6) {
            // Tags are stacked vertically in the layout (no spread top-down) - can't get rotation from them
            Tag strongest = tags.get(0);
            for (Tag t : tags) if (t.weight > strongest.weight) strongest = t;
            theta = singleTagTheta(strongest);
        } else {
            theta = Math.atan2(sCross, sDot);
        }

        double cos = Math.cos(theta), sin = Math.sin(theta);
        // opening (0,0) -> camera:  c_bar - R * a_bar
        double ox = cx - (cos * ax - sin * bx);
        double ow = cw - (sin * ax + cos * bx);

        // Height: average of (tag height - layout height)
        double ou = 0;
        for (Tag t : tags) ou += t.weight * (t.u - t.layoutPos[2]);
        ou /= sw;

        return pointToAim(ox, -ow, ou, Method.LAYOUT_FIT);
    }

    private double singleTagTheta(Tag t) {
        return useSingleTagYaw ? Math.toRadians(singleTagYawSign * t.yawDeg) : 0;
    }

    private double[] singleTag(Tag t) {
        double theta = singleTagTheta(t);
        double cos = Math.cos(theta), sin = Math.sin(theta);
        double a = t.layoutPos[0], b = t.layoutPos[1];
        double ox = t.x - (cos * a - sin * b);
        double ow = -t.f - (sin * a + cos * b);
        double ou = t.u - t.layoutPos[2];
        return pointToAim(ox, -ow, ou, Method.SINGLE_TAG);
    }

    private double[] clusterAverage(List<Tag> cluster, boolean all3d) {
        double sw = 0;
        for (Tag t : cluster) sw += t.weight;

        if (all3d) {
            double x = 0, f = 0, u = 0;
            for (Tag t : cluster) {
                x += t.weight * t.x;
                f += t.weight * t.f;
                u += t.weight * t.u;
            }
            return pointToAim(x / sw, f / sw, u / sw, Method.CLUSTER_AVERAGE);
        }

        double txAvg = 0, tyAvg = 0;
        for (Tag t : cluster) {
            txAvg += t.weight * t.txDeg;
            tyAvg += t.weight * t.tyDeg;
        }
        return new double[]{txAvg / sw, tyAvg / sw, -1, 0, Method.CLUSTER_AVERAGE.ordinal()};
    }

    /** Point in leveled turret frame (x right, f forward, u up, inches) -> {tx, ty, dist, height, method}. */
    private double[] pointToAim(double x, double f, double u, Method m) {
        if (f <= 0.5) return null; // behind / on top of the camera, bad solve
        double dist = Math.hypot(x, f);
        double txDeg = Math.toDegrees(Math.atan2(x, f));
        double tyDeg = Math.toDegrees(Math.atan2(u, dist));
        return new double[]{txDeg, tyDeg, dist, u, m.ordinal()};
    }

    // ------------------------------------------------------------------------------------------
    // Output handling
    // ------------------------------------------------------------------------------------------

    private void applyResult(double[] aim, List<Tag> cluster) {
        double a = hadValue ? clamp(smoothing, 0, 1) : 1;
        tx = tx + (aim[0] - tx) * a;
        ty = ty + (aim[1] - ty) * a;
        if (aim[2] < 0) {
            distanceIn = -1;
        } else {
            distanceIn = (hadValue && distanceIn >= 0) ? distanceIn + (aim[2] - distanceIn) * a : aim[2];
        }
        heightIn = heightIn + (aim[3] - heightIn) * a;

        method = Method.values()[(int) aim[4]];
        tagsUsed = cluster.size();
        hiveName = cluster.get(0).hive != null ? cluster.get(0).hive.name : "unknown";
        tagIds = new ArrayList<>();
        for (Tag t : cluster) tagIds.add(t.id);

        hadValue = true;
        hasTarget = true;
        lastSeenMs = System.currentTimeMillis();
    }

    private boolean finishNoTags() {
        if (hadValue && System.currentTimeMillis() - lastSeenMs <= holdTimeMs) {
            method = Method.HELD;
            hasTarget = true;
            return true;
        }
        hadValue = false;
        hasTarget = false;
        method = Method.NONE;
        tagsUsed = 0;
        tagIds = new ArrayList<>();
        return false;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Clear smoothing / held values (e.g. when switching alliance or hive). */
    public void reset() {
        hadValue = false;
        hasTarget = false;
        method = Method.NONE;
        tagsUsed = 0;
        tagIds = new ArrayList<>();
        tx = ty = heightIn = 0;
        distanceIn = -1;
    }

    // ------------------------------------------------------------------------------------------
    // Getters
    // ------------------------------------------------------------------------------------------

    public boolean hasTarget() { return hasTarget; }

    /** Horizontal angle to the opening in degrees. + = right. Same convention as Limelight tx. */
    public double getTx() { return tx; }

    /** Vertical angle to the opening in degrees above level (Limelight ty if there is no 3D data). */
    public double getTy() { return ty; }

    /** Horizontal distance from turret pivot to the opening in inches, or -1 if no 3D data. */
    public double getDistanceInches() { return distanceIn; }

    /** Height of the opening relative to the camera, inches. Only meaningful with 3D data. */
    public double getHeightInches() { return heightIn; }

    public int getTagsUsed() { return tagsUsed; }

    public List<Integer> getTagIds() { return tagIds; }

    public String getHiveName() { return hiveName; }

    public Method getMethod() { return method; }

    public void addTelemetry(Telemetry telemetry) {
        if (telemetry == null) return;
        telemetry.addLine("--- Hive Cluster Targeting ---");
        telemetry.addData("Has target", hasTarget);
        telemetry.addData("Method", method);
        telemetry.addData("Hive", hiveName);
        telemetry.addData("Tags used", tagsUsed + " " + tagIds);
        telemetry.addData("Opening tx", "%.2f", tx);
        telemetry.addData("Opening ty", "%.2f", ty);
        telemetry.addData("Opening dist (in)", "%.1f", distanceIn);
        telemetry.addData("Camera pitch (deg)", "%.1f%s", cameraPitchDeg, isTiltSettling() ? " (moving)" : "");
    }
}
