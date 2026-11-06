package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class VisionSubsystem {

    public enum Goal {
        AUDIENCE,
        FAR
    }

    public enum SelectionMode {
        MOST_TAGS,
        FIELD_HALF,
        AUDIENCE_ONLY,
        FAR_ONLY
    }

    public enum CameraFrame {
        LEGACY_OPTICAL,
        NWU
    }

    public static int PIPELINE = 0;

    public static CameraFrame CAMERA_FRAME =
            CameraFrame.LEGACY_OPTICAL;

    // Positive means the camera's optical axis points upward.
    public static double CAMERA_PITCH_UP_DEG = 40.0;

    public static long MAX_STALENESS_MS = 250;
    public static double MIN_RANGE_M = 0.15;
    public static double MAX_RANGE_M = 10.0;

    // Used only by FIELD_HALF selection.
    public static double FIELD_SPLIT_Y_IN = 72.0;
    public static Goal LOWER_HALF_GOAL = Goal.AUDIENCE;
    public static Goal UPPER_HALF_GOAL = Goal.FAR;

    public static double HIVE_WAIT_SECONDS = 2.0;

    private final Limelight3A limelight;

    private final ElapsedTime hiveTimer = new ElapsedTime();

    private boolean hiveSeen = false;
    private boolean hiveVisible = false;
    private String hiveVisibleIds = "";
    private String hiveFirstSeenIds = "";

    // Track which hive the detection latch belongs to.
    private Goal trackedHiveGoal = null;
    private boolean trackedHiveIsBlue = true;

    public static final class AimObservation {

        public final Goal goal;
        public final int tagCount;
        public final int audienceCount;
        public final int farCount;
        public final String visibleIds;

        // Robot-relative forward/left/up, in meters from the camera.
        public final double forwardM;
        public final double leftM;
        public final double upM;
        public final double totalArea;

        private AimObservation(
                Goal goal,
                int tagCount,
                int audienceCount,
                int farCount,
                String visibleIds,
                double forwardM,
                double leftM,
                double upM,
                double totalArea
        ) {
            this.goal = goal;
            this.tagCount = tagCount;
            this.audienceCount = audienceCount;
            this.farCount = farCount;
            this.visibleIds = visibleIds;
            this.forwardM = forwardM;
            this.leftM = leftM;
            this.upM = upM;
            this.totalArea = totalArea;
        }

        public double horizontalDistanceM() {
            return Math.hypot(forwardM, leftM);
        }
    }

    private static final class Accumulator {

        private final boolean[] seen = new boolean[46];

        private int count;
        private double forward;
        private double left;
        private double up;
        private double area;

        private final StringBuilder ids = new StringBuilder();

        private void add(
                int id,
                double f,
                double l,
                double u,
                double a
        ) {
            if (id < 0 || id >= seen.length || seen[id]) {
                return;
            }

            seen[id] = true;
            count++;

            forward += f;
            left += l;
            up += u;

            if (Double.isFinite(a) && a > 0) {
                area += a;
            }

            if (ids.length() > 0) {
                ids.append(',');
            }

            ids.append(id);
        }

        private AimObservation observation(
                Goal goal,
                int audienceCount,
                int farCount
        ) {
            if (count == 0) {
                return null;
            }

            return new AimObservation(
                    goal,
                    count,
                    audienceCount,
                    farCount,
                    ids.toString(),
                    forward / count,
                    left / count,
                    up / count,
                    area
            );
        }
    }

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        resetHiveDetection();
    }

    // Preserves compatibility with your existing Blue auto-aim code.
    public AimObservation getBlueGoal(
            SelectionMode mode,
            double robotYIn
    ) {
        return getGoal(mode, robotYIn, true);
    }

    public AimObservation getRedGoal(
            SelectionMode mode,
            double robotYIn
    ) {
        return getGoal(mode, robotYIn, false);
    }

    /**
     * Returns a fresh usable observation, or null.
     *
     * Tag mapping:
     * Red far:       30-33
     * Red audience:  34-37
     * Blue audience: 38-41
     * Blue far:      42-45
     *
     * Vectors average the visible tag centers.
     */
    public AimObservation getGoal(
            SelectionMode mode,
            double robotYIn,
            boolean isBlue
    ) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        long staleness = result.getStaleness();

        if (staleness < 0 || staleness > MAX_STALENESS_MS) {
            return null;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        if (tags == null || tags.isEmpty()) {
            return null;
        }

        Accumulator audience = new Accumulator();
        Accumulator far = new Accumulator();

        final double pitch = Math.toRadians(CAMERA_PITCH_UP_DEG);
        final double cos = Math.cos(pitch);
        final double sin = Math.sin(pitch);

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();

            if (isBlue) {
                if (id < 38 || id > 45) {
                    continue;
                }
            } else {
                if (id < 30 || id > 37) {
                    continue;
                }
            }

            Pose3D target = tag.getTargetPoseCameraSpace();

            if (target == null || target.getPosition() == null) {
                continue;
            }

            double x = target.getPosition().x;
            double y = target.getPosition().y;
            double z = target.getPosition().z;

            if (!Double.isFinite(x)
                    || !Double.isFinite(y)
                    || !Double.isFinite(z)) {
                continue;
            }

            double f;
            double l;
            double u;

            if (CAMERA_FRAME == CameraFrame.LEGACY_OPTICAL) {
                // Camera axes: right/down/forward.
                f = z * cos + y * sin;
                l = -x;
                u = z * sin - y * cos;
            } else {
                // Camera axes: forward/left/up.
                f = x * cos - z * sin;
                l = y;
                u = x * sin + z * cos;
            }

            double range = Math.sqrt(f * f + l * l + u * u);

            if (!Double.isFinite(range)
                    || range < MIN_RANGE_M
                    || range > MAX_RANGE_M) {
                continue;
            }

            if (f <= 0.05) {
                continue;
            }

            boolean isAudience = isBlue
                    ? id <= 41
                    : id >= 34;

            if (isAudience) {
                audience.add(id, f, l, u, tag.getTargetArea());
            } else {
                far.add(id, f, l, u, tag.getTargetArea());
            }
        }

        AimObservation audienceObservation = audience.observation(
                Goal.AUDIENCE,
                audience.count,
                far.count
        );

        AimObservation farObservation = far.observation(
                Goal.FAR,
                audience.count,
                far.count
        );

        if (mode == null) {
            mode = SelectionMode.MOST_TAGS;
        }

        switch (mode) {
            case AUDIENCE_ONLY:
                return audienceObservation;

            case FAR_ONLY:
                return farObservation;

            case FIELD_HALF:
                if (!Double.isFinite(robotYIn)) {
                    return null;
                }

                Goal preferred = robotYIn < FIELD_SPLIT_Y_IN
                        ? LOWER_HALF_GOAL
                        : UPPER_HALF_GOAL;

                return preferred == Goal.AUDIENCE
                        ? audienceObservation
                        : farObservation;

            case MOST_TAGS:
            default:
                if (audienceObservation == null) {
                    return farObservation;
                }

                if (farObservation == null) {
                    return audienceObservation;
                }

                if (audienceObservation.tagCount
                        != farObservation.tagCount) {
                    return audienceObservation.tagCount
                            > farObservation.tagCount
                            ? audienceObservation
                            : farObservation;
                }

                return audienceObservation.totalArea
                        >= farObservation.totalArea
                        ? audienceObservation
                        : farObservation;
        }
    }

    // Existing one-argument calls continue to use Blue.
    public boolean hiveReady(Goal goal) {
        return hiveReady(goal, true);
    }

    /**
     * Call once per loop.
     *
     * Latches the first usable observation of the selected hive.
     * Returns true after HIVE_WAIT_SECONDS, even if tags disappear.
     *
     * This detects tag visibility, not physical hive tilt.
     */
    public boolean hiveReady(Goal goal, boolean isBlue) {
        if (goal == null) {
            throw new IllegalArgumentException("Hive goal cannot be null");
        }

        // Prevent a previous hive's detection from enabling a new hive.
        if (trackedHiveGoal != goal || trackedHiveIsBlue != isBlue) {
            resetHiveDetection();
            trackedHiveGoal = goal;
            trackedHiveIsBlue = isBlue;
        }

        SelectionMode mode = goal == Goal.AUDIENCE
                ? SelectionMode.AUDIENCE_ONLY
                : SelectionMode.FAR_ONLY;

        AimObservation observation = getGoal(mode, 0, isBlue);

        hiveVisible = observation != null;
        hiveVisibleIds = hiveVisible ? observation.visibleIds : "";

        if (!hiveSeen && hiveVisible) {
            hiveSeen = true;
            hiveFirstSeenIds = hiveVisibleIds;
            hiveTimer.reset();
        }

        return hiveSeen
                && hiveTimer.seconds() >= HIVE_WAIT_SECONDS;
    }

    public void resetHiveDetection() {
        hiveSeen = false;
        hiveVisible = false;
        hiveVisibleIds = "";
        hiveFirstSeenIds = "";

        trackedHiveGoal = null;
        trackedHiveIsBlue = true;

        hiveTimer.reset();
    }

    public boolean isHiveVisible() {
        return hiveVisible;
    }

    public boolean hasSeenHive() {
        return hiveSeen;
    }

    public String getHiveVisibleIds() {
        return hiveVisibleIds;
    }

    public String getHiveFirstSeenIds() {
        return hiveFirstSeenIds;
    }

    public double getHiveElapsedSeconds() {
        return hiveSeen ? hiveTimer.seconds() : 0.0;
    }

    public double getHiveWaitRemainingSeconds() {
        return hiveSeen
                ? Math.max(
                0.0,
                HIVE_WAIT_SECONDS - hiveTimer.seconds()
        )
                : HIVE_WAIT_SECONDS;
    }

    public void stop() {
        limelight.stop();
    }
}