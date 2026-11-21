package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.content.SharedPreferences;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.common.HardwareNames;
import org.firstinspires.ftc.teamcode.common.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

public class TurretCamera extends Subsystem {
    public enum Alliance { RED, BLUE, UNKNOWN }

    public enum Cell {
        RED_SCORING, RED_AUDIENCE, BLUE_AUDIENCE, BLUE_SCORING, UNKNOWN
    }

    private static final long MAX_AGE_NANOS = 200_000_000L;
    private static final int REQUIRED_FRAMES = 2;
    private static final double UPRIGHT_ROLL_LIMIT_DEGREES = 90;

    private AprilTagProcessor aprilTags;
    private VisionPortal portal;
    private VisionPortal.CameraState cameraState;
    private final List<String> observations = new ArrayList<>();
    private int detectionCount;
    private Alliance alliance = Alliance.UNKNOWN;
    private Cell candidate = Cell.UNKNOWN;
    private Cell upwardCell = Cell.UNKNOWN;
    private Cell pending = Cell.UNKNOWN;
    private int agreeingFrames;
    private boolean missedFrame;
    private long lastUpwardNanos;
    private long observationNanos;
    private AprilTagPoseFtc targetPose;
    private String reason = "Waiting for camera";
    private String pendingCapture;
    private final SharedPreferences lightingPreferences;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    private ExposureControl.Mode defaultExposureMode;
    private String lightingStatus = "Waiting for webcam";
    private long lastLightingReadNanos;
    private long displayedExposureMs;
    private int displayedGain;
    private ExposureControl.Mode displayedExposureMode;

    public TurretCamera(HardwareMap hardwareMap) {
        lightingPreferences = hardwareMap.appContext.getSharedPreferences("hive_camera_lighting", 0);
        WebcamName camera = hardwareMap.tryGet(WebcamName.class, HardwareNames.hive_camera);
        if (camera == null) return;

        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        portal = new VisionPortal.Builder().setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTags).build();
    }

    @Override
    public void periodic() {
        cameraState = portal == null ? null : portal.getCameraState();
        if (cameraState != VisionPortal.CameraState.STREAMING) {
            clearObservation(cameraState == null ? "Camera missing" : "Camera not streaming");
            resetAgreement();
            return;
        }
        if (exposureControl == null) {
            exposureControl = portal.getCameraControl(ExposureControl.class);
            gainControl = portal.getCameraControl(GainControl.class);
            defaultExposureMode = exposureControl.getMode();
            lightingStatus = "Using webcam defaults";
            if (lightingPreferences.contains("exposureMs")) {
                applyLighting(lightingPreferences.getLong("exposureMs", 0),
                        lightingPreferences.getInt("gain", 0));
            } else if (lightingPreferences.getBoolean("automatic", false)) {
                resetLighting();
            }
        }
        long now = System.nanoTime();
        // Camera-control reads can involve USB traffic. Refresh at telemetry speed.
        if (displayedExposureMode == null || now - lastLightingReadNanos >= 250_000_000L) {
            displayedExposureMode = exposureControl.getMode();
            displayedExposureMs = exposureControl.getExposure(TimeUnit.MILLISECONDS);
            displayedGain = gainControl.getGain();
            lastLightingReadNanos = now;
        }

        // The robot loop can run several times per camera frame. The SDK returns null
        // between frames and an empty list when a new frame contains no detections.
        List<AprilTagDetection> fresh = aprilTags.getFreshDetections();
        now = System.nanoTime();
        if (pending != Cell.UNKNOWN && !isFresh(lastUpwardNanos, now)) {
            resetAgreement();
        }
        if (observationNanos != 0 && !isFresh(observationNanos, now)) {
            clearObservation("Camera observation expired");
        }
        if (fresh != null) {
            evaluateFrame(fresh, now);
            if (pendingCapture != null) {
                // Log the first processed observation after the capture request.
                RobotLog.ii("HiveAimCapture", "%s next observation: alliance=%s candidate=%s "
                                + "upward=%s agreement=%d detections=%d reason=%s clusters=%s",
                        pendingCapture, alliance, candidate, upwardCell, agreeingFrames,
                        detectionCount, reason, observations);
                pendingCapture = null;
            }
        }
    }

    private void evaluateFrame(List<AprilTagDetection> detections, long now) {
        clearObservation("No fresh cell detected");
        detectionCount = detections.size();
        observationNanos = now;
        Cell visibleCell = Cell.UNKNOWN;
        AprilTagClusterDetection upward = null;
        boolean multipleVisible = false;
        boolean multipleUpward = false;
        boolean pendingIsDownward = false;

        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            Cell cell = cellOf(cluster);
            String rejection = rejectionReason(cluster, cell, now);
            if (rejection != null) {
                observations.add(describe(cluster, rejection, now));
                continue;
            }

            // A candidate is a recognized cell for our alliance, even when its pose
            // is missing or its tags are upside down.
            if (visibleCell != Cell.UNKNOWN && visibleCell != cell) multipleVisible = true;
            visibleCell = cell;
            String poseProblem = poseProblem(cluster.ftcPose);
            if (poseProblem != null) {
                observations.add(describe(cluster, poseProblem, now));
                continue;
            }

            // FIRST's BIOBUZZ convention uses image roll: the tag cluster looks
            // upright on the upward-facing cell when the webcam is mounted upright.
            if (Math.abs(cluster.ftcPose.roll) >= UPRIGHT_ROLL_LIMIT_DEGREES) {
                pendingIsDownward |= cell == pending;
                observations.add(describe(cluster, "Downward (roll)", now));
                continue;
            }

            observations.add(describe(cluster, "Upward (roll)", now));
            if (upward != null && cellOf(upward) != cell) multipleUpward = true;
            if (upward == null || cluster.percentClusterFound > upward.percentClusterFound) {
                upward = cluster;
            }
        }

        candidate = multipleVisible ? Cell.UNKNOWN : visibleCell;
        if (multipleUpward) {
            resetAgreement();
            reason = "Ambiguous: multiple upward cells";
        } else if (upward != null) {
            candidate = upwardCell = cellOf(upward);
            observationNanos = upward.frameAcquisitionNanoTime;
            confirmFrame(upward);
        } else {
            // Allow one missed frame within the age limit. Drop the aiming pose
            // immediately and discard prior agreement on contradictory evidence.
            if (pendingIsDownward || missedFrame) resetAgreement();
            missedFrame = pending != Cell.UNKNOWN;
            reason = visibleCell == Cell.UNKNOWN
                    ? "No fresh cell for selected alliance" : "No usable upward pose";
        }
    }

    private static boolean isFresh(long frameNanos, long now) {
        long age = now - frameNanos;
        return frameNanos > 0 && age >= 0 && age <= MAX_AGE_NANOS;
    }

    private String rejectionReason(AprilTagClusterDetection cluster, Cell cell, long now) {
        if (!isFresh(cluster.frameAcquisitionNanoTime, now)) return "Stale or invalid timestamp";
        if (cell == Cell.UNKNOWN) return "Unknown cluster name";
        if (alliance == Alliance.UNKNOWN) return "Select alliance";
        if (!cell.name().startsWith(alliance.name() + "_")) return "Wrong alliance";
        return null;
    }

    private static String poseProblem(AprilTagPoseFtc pose) {
        if (pose == null) return "No pose";
        if (!Double.isFinite(pose.roll) || !Double.isFinite(pose.bearing)
                || !Double.isFinite(pose.range) || pose.range <= 0) return "Invalid pose";
        return null;
    }

    private static Cell cellOf(AprilTagClusterDetection cluster) {
        if (cluster.metadata == null || cluster.metadata.shortName == null) return Cell.UNKNOWN;

        switch (cluster.metadata.shortName.trim().toUpperCase(Locale.US)) {
            case "RED SCORING": return Cell.RED_SCORING;
            case "RED AUDIENCE": return Cell.RED_AUDIENCE;
            case "BLUE AUDIENCE": return Cell.BLUE_AUDIENCE;
            case "BLUE SCORING": return Cell.BLUE_SCORING;
            default: return Cell.UNKNOWN;
        }
    }

    private void confirmFrame(AprilTagClusterDetection observation) {
        if (upwardCell != pending) {
            resetAgreement();
            pending = upwardCell;
        }

        // Count only newer acquisition timestamps. One fresh pose identifies the
        // upward cell for diagnostics. A second agreeing pose enables aiming.
        if (observation.frameAcquisitionNanoTime > lastUpwardNanos) {
            agreeingFrames = Math.min(REQUIRED_FRAMES, agreeingFrames + 1);
            lastUpwardNanos = observation.frameAcquisitionNanoTime;
            missedFrame = false;
        }
        if (agreeingFrames >= REQUIRED_FRAMES) {
            targetPose = observation.ftcPose;
            reason = "Upward target confirmed";
        } else {
            reason = "Waiting for another agreeing frame";
        }
    }

    private void resetAgreement() {
        pending = Cell.UNKNOWN;
        agreeingFrames = 0;
        lastUpwardNanos = 0;
        missedFrame = false;
    }

    private void clearObservation(String why) {
        candidate = upwardCell = Cell.UNKNOWN;
        targetPose = null;
        observationNanos = 0;
        detectionCount = 0;
        observations.clear();
        reason = why;
    }

    public void setAlliance(Alliance value) {
        Alliance selected = value == null ? Alliance.UNKNOWN : value;
        if (selected == alliance) return;
        alliance = selected;
        clearObservation("Alliance changed; waiting for a camera frame");
        resetAgreement();
    }

    private static String describe(AprilTagClusterDetection cluster, String result, long now) {
        String name = cluster.metadata == null ? "Unknown cluster" : cluster.metadata.shortName;
        String summary = String.format(Locale.US, "%s: %s | age %.1f ms | tags %.0f%%",
                name, result, (now - cluster.frameAcquisitionNanoTime) / 1e6,
                (double) cluster.percentClusterFound);
        AprilTagPoseFtc pose = cluster.ftcPose;
        if (pose == null) return summary;

        return summary + String.format(Locale.US, " | range %.1f in | bearing %.1f° | roll %.1f°",
                pose.range, pose.bearing, pose.roll);
    }

    @Override
    public void writeTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera", cameraState == null ? "MISSING" : cameraState);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Candidate", candidate);
        telemetry.addData("Upward cell", upwardCell);
        telemetry.addData("Agreement", "%d / %d", agreeingFrames, REQUIRED_FRAMES);
        telemetry.addData("Vision target available", hasUpwardTarget());
        telemetry.addData("Selection reason", reason);
        telemetry.addData("Detections (unfiltered)", detectionCount);
        for (String observation : observations) {
            telemetry.addLine(observation);
        }
    }

    public boolean hasUpwardTarget() {
        return targetPose != null && isFresh(observationNanos, System.nanoTime());
    }

    public Cell getTargetCell() {
        return hasUpwardTarget() ? upwardCell : Cell.UNKNOWN;
    }

    public double getBearingDegrees() {
        return hasUpwardTarget() ? targetPose.bearing : Double.NaN;
    }

    public double getRangeInches() {
        return hasUpwardTarget() ? targetPose.range : Double.NaN;
    }

    public String requestRawFrame() {
        if (portal == null || portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return null;
        }

        // VisionPortal saves the next raw image to shared storage with its own prefix.
        String name = "HiveAim-" + System.currentTimeMillis();
        portal.saveNextFrameRaw(name);
        pendingCapture = "VisionPortal-" + name + ".png";
        RobotLog.ii("HiveAimCapture", "Requested %s", pendingCapture);
        return pendingCapture;
    }

    public void adjustLighting(long exposureStepMs, int gainStep) {
        if (!lightingReady()) return;
        applyLighting(exposureControl.getExposure(TimeUnit.MILLISECONDS) + exposureStepMs,
                gainControl.getGain() + gainStep);
        // Require new agreeing frames after the image settings change.
        resetAgreement();
        clearObservation("Lighting changed");
    }

    private void applyLighting(long exposureMs, int gain) {
        if (!exposureControl.isModeSupported(ExposureControl.Mode.Manual)
                || !exposureControl.isExposureSupported()) {
            lightingStatus = "Manual exposure unsupported";
            return;
        }
        long minMs = Math.max(1, TimeUnit.NANOSECONDS.toMillis(
                exposureControl.getMinExposure(TimeUnit.NANOSECONDS) + 999_999));
        long maxMs = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
        if (minMs > maxMs) {
            lightingStatus = "No supported whole-millisecond exposure";
            return;
        }
        long boundedExposure = Math.max(minMs, Math.min(maxMs, exposureMs));
        int boundedGain = Math.max(gainControl.getMinGain(), Math.min(gainControl.getMaxGain(), gain));
        if (exposureControl.getMode() != ExposureControl.Mode.Manual
                && !exposureControl.setMode(ExposureControl.Mode.Manual)) {
            lightingStatus = "Camera rejected manual mode";
            return;
        }
        boolean exposureApplied = exposureControl.setExposure(boundedExposure, TimeUnit.MILLISECONDS);
        boolean gainApplied = gainControl.setGain(boundedGain);
        lightingStatus = exposureApplied && gainApplied
                ? "Manual settings applied" : "Camera rejected exposure or gain";
    }

    public void resetLighting() {
        if (!lightingReady()) return;
        if (defaultExposureMode == ExposureControl.Mode.Manual
                || defaultExposureMode == ExposureControl.Mode.Unknown) {
            // Some webcams report AperturePriority instead of Auto.
            defaultExposureMode = exposureControl.isModeSupported(ExposureControl.Mode.AperturePriority)
                    ? ExposureControl.Mode.AperturePriority : ExposureControl.Mode.Auto;
        }
        if (exposureControl.getMode() != defaultExposureMode
                && !exposureControl.setMode(defaultExposureMode)) {
            lightingStatus = "Camera rejected automatic exposure";
            return;
        }
        resetAgreement();
        clearObservation("Lighting changed");
        lightingStatus = "Automatic exposure active";
    }

    public void saveLighting() {
        if (!lightingReady()) return;
        SharedPreferences.Editor editor = lightingPreferences.edit();
        if (exposureControl.getMode() == ExposureControl.Mode.Manual) {
            editor.putLong("exposureMs", exposureControl.getExposure(TimeUnit.MILLISECONDS))
                    .putInt("gain", gainControl.getGain())
                    .remove("automatic");
        } else {
            editor.remove("exposureMs").remove("gain").putBoolean("automatic", true);
        }
        editor.apply();
        lightingStatus = "Saved for future OpModes on this hub";
    }

    private boolean lightingReady() {
        boolean ready = cameraState == VisionPortal.CameraState.STREAMING && exposureControl != null;
        if (!ready) lightingStatus = "Waiting for webcam to stream";
        return ready;
    }

    public void writeLightingTelemetry(Telemetry telemetry) {
        telemetry.addData("Lighting", lightingStatus);
        if (cameraState == VisionPortal.CameraState.STREAMING && displayedExposureMode != null) {
            telemetry.addData("Exposure / gain", "%s / %d ms / %d", displayedExposureMode,
                    displayedExposureMs, displayedGain);
            telemetry.addData("Camera FPS", "%.1f", portal.getFps());
        }
    }

    public void shutdown() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
        cameraState = null;
        clearObservation("Camera closed");
        resetAgreement();
    }
}
