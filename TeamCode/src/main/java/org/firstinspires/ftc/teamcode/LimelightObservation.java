package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.ArrayList;
import java.util.List;

/** SDK adapter. Reads only per-tag t6t_cs, never botpose or an assumed tag-height range. */
public final class LimelightObservation {
    /**
     * Physical Limelight frame (2026-11-05, raised Red far tags 30-33 seen head-on at about 1.12 m):
     * the camera reports tag +Z pointing INTO the print (raw facing scores -0.96 to -0.99).
     * Robot math uses an outward +Z, so Rx(pi) is applied here once: for the SDK's intrinsic
     * Z-Y-X angles that is exactly roll + pi. SimLoop's raw camera agrees.
     */
    public static final boolean LIMELIGHT_TAG_Z_INTO_PRINT = true;
    /**
     * The Limelight is mounted upside down on the turret. The web UI "Stream Orientation: Upside-Down"
     * rotates only the video stream: its ID labels are drawn upside down, and the per-tag 3D camera-space
     * poses stay in the inverted sensor frame. Physical frame 2026-11-13 (Red audience, tags 34-36 and
     * 38-39 at about 1.2 m, 1280x960): in the upright stream the tags sit about 7 deg below center while
     * the raw pose put them 7.3 deg above; raw blue tags were camera-left of red although the blue hive
     * is at +X (camera-right when looking +Y); the raw tag 36-to-38 spacing (16.3 in) matches the map
     * only after this flip. Raw values computed 1.419 m / 41.6 deg against a field-measured 1.21 m /
     * 29.9 deg; the flip gives 1.242 m / 28.4 deg. Rz(pi) about the optical axis is applied here once:
     * right and down are negated and, for the SDK's intrinsic Z-Y-X angles, yaw + pi.
     * The 2026-11-05 frame was nearly centered, which hid the vertical error there.
     */
    public static final boolean LIMELIGHT_MOUNTED_INVERTED = true;

    /** Converts an SDK tag roll into the robot's outward-+Z tag basis. */
    public static double robotTagRoll(double sdkRollRadians) {
        return LIMELIGHT_TAG_Z_INTO_PRINT ? RobotMath.wrap(sdkRollRadians + Math.PI) : sdkRollRadians;
    }
    /** Converts an SDK tag yaw from the inverted sensor frame into the upright camera frame. */
    public static double robotTagYaw(double sdkYawRadians) {
        return LIMELIGHT_MOUNTED_INVERTED ? RobotMath.wrap(sdkYawRadians + Math.PI) : sdkYawRadians;
    }
    /** Converts a sensor-frame right/down coordinate into the upright camera frame. */
    public static double robotCameraLateral(double sensorMeters) {
        return LIMELIGHT_MOUNTED_INVERTED ? -sensorMeters : sensorMeters;
    }
    /** The single SDK-to-robot conversion: raw t6t_cs (sensor frame) to an upright, outward-+Z observation. */
    public static TagObservation robotTag(int id, double sensorRight, double sensorDown, double forward,
                                          double sdkRoll, double sdkPitch, double sdkYaw, double area) {
        return new TagObservation(id, robotCameraLateral(sensorRight), robotCameraLateral(sensorDown), forward,
                robotTagRoll(sdkRoll), sdkPitch, robotTagYaw(sdkYaw), area);
    }

    public final double frame, captureTime;
    /** Delay components in ms: time since the Control Hub parsed the result, then Limelight-reported image capture and processing. */
    public final double hubWaitMs, captureLatencyMs, processingLatencyMs;
    /** Distinct camera frames per second seen by the caller's polling (NaN when not tracked). */
    public final double newFrameRateHz;
    public final String rejectionReason;
    public final List<TagObservation> tags = new ArrayList<>();
    public LimelightObservation(double now, long wallNowMs, LLResult result) {
        this(now, wallNowMs, result, Double.NaN);
    }
    public LimelightObservation(double now, long wallNowMs, LLResult result, double newFrameRateHz) {
        this.newFrameRateHz = newFrameRateHz;
        hubWaitMs = result == null ? Double.NaN : result.getStaleness();
        captureLatencyMs = result == null ? Double.NaN : result.getCaptureLatency();
        processingLatencyMs = result == null ? Double.NaN : result.getTargetingLatency();
        frame = result == null ? Double.NaN : result.getTimestamp();
        captureTime = result == null ? Double.NaN : PoseHistory.captureTimeFromAge(now, result.getStaleness(),
                result.getCaptureLatency(), result.getTargetingLatency(),
                ShootingConfiguration.ADDITIONAL_VISION_TRANSPORT_SECONDS);
        rejectionReason = result == null ? "No Limelight result"
                : !result.isValid() ? "No valid camera detection"
                : result.getStaleness() < 0 ? "Invalid camera receipt age"
                : result.getStaleness() > LimelightTagSubsystem.MAX_STALENESS_MS ? "Stale camera result"
                : !RobotMath.finite(frame,captureTime) ? "Invalid camera timestamp / capture timing" : null;
        if (result == null || !result.isValid() || result.getStaleness() < 0
                || result.getStaleness() > LimelightTagSubsystem.MAX_STALENESS_MS) return;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag == null) continue;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null) { tags.add(new TagObservation(tag.getFiducialId(),0,0,0,0,0,0,0)); continue; }
            Position p = pose.getPosition().toUnit(DistanceUnit.METER);
            tags.add(robotTag(tag.getFiducialId(), p.x, p.y, p.z,
                    pose.getOrientation().getRoll(AngleUnit.RADIANS), pose.getOrientation().getPitch(AngleUnit.RADIANS),
                    pose.getOrientation().getYaw(AngleUnit.RADIANS), tag.getTargetArea()));
        }
    }

    /** Counts distinct camera frame timestamps over the last second; the SDK re-polls the same frame every 10 ms. */
    public static final class FrameRate {
        private final java.util.ArrayDeque<Double> seen = new java.util.ArrayDeque<>();
        private double lastFrame = Double.NaN;
        public double update(LLResult result, double now) {
            if (result != null && Double.isFinite(result.getTimestamp()) && result.getTimestamp() != lastFrame) {
                lastFrame = result.getTimestamp(); seen.addLast(now);
            }
            while (!seen.isEmpty() && now - seen.peekFirst() > 1.0) seen.removeFirst();
            return seen.size() < 2 ? Double.NaN : (seen.size() - 1) / Math.max(1e-3, seen.peekLast() - seen.peekFirst());
        }
    }
}
