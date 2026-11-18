package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.common.HardwareNames;
import org.firstinspires.ftc.teamcode.common.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

public class TurretCamera extends Subsystem {
    private AprilTagProcessor aprilTags;
    private VisionPortal portal;
    private List<AprilTagDetection> detections = Collections.emptyList();

    public enum Alliance { RED, BLUE, UNKNOWN }
    public enum Cell {
        RED_SCORING,RED_AUDIENCE, BLUE_AUDIENCE, BLUE_SCORING, UNKNOWN}
    private static final double MAX_AGE_MS = 200;
    private static final int REQUIRED_FRAMES = 3;
    private Alliance alliance = Alliance.UNKNOWN;
    private Cell candidate = Cell.UNKNOWN;
    private Cell confirmed = Cell.UNKNOWN;
    private Cell pending = Cell.UNKNOWN;
    private int agreeingFrames = 0;
    private long lastFrameNanos = -1;
    private double bearing = Double.NaN, range = Double.NaN;
    private String reason = "Not evaluated";

    public TurretCamera(HardwareMap hardwareMap) {
        WebcamName camera = hardwareMap.tryGet(WebcamName.class, HardwareNames.hive_camera);
        if (camera == null) return;
        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
        portal = new VisionPortal.Builder().setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTags).build();
    }
    public void periodic() {
        candidate = confirmed = Cell.UNKNOWN;
        bearing = range = Double.NaN;
        detections = Collections.emptyList();
        if (portal == null || aprilTags == null) {
            lose("Camera missing"); return;
        }
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            lose("Camera not streaming"); return;
        }
        List<AprilTagDetection> latest = aprilTags.getDetections();
        if (latest != null) detections = latest;
        AprilTagClusterDetection chosen = null;
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection c = (AprilTagClusterDetection) detection;
            if (rejectionReason(c) != null) continue;
            if (chosen != null && cellOf(chosen) != cellOf(c)) {
                lose("Ambiguous: two cells qualify"); return;
            }
            if (chosen == null || c.frameAcquisitionNanoTime
                    > chosen.frameAcquisitionNanoTime) chosen = c;
        }
        if (chosen == null) {
            lose("No eligible cell; inspect rejection reasons"); return;
        }
        candidate = cellOf(chosen);
        bearing = chosen.ftcPose.bearing;
        range = chosen.ftcPose.range;
        confirmFrame(chosen.frameAcquisitionNanoTime);
    }

    public void writeTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera", portal == null ? "MISSING" : portal.getCameraState());
        telemetry.addData("Detections (unfiltered)", detections.size());
        for (AprilTagDetection detection : detections) {
            double ageMs = (System.nanoTime() - detection.frameAcquisitionNanoTime) / 1e6;
            telemetry.addData("Observation age ms", ageMs);
            if (ageMs > 200 || !(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cell = (AprilTagClusterDetection) detection;
            telemetry.addData("Fresh cell", cell.metadata.shortName);
            telemetry.addData("Visible tags %", cell.percentClusterFound);
            if (cell.ftcPose == null) continue;
            telemetry.addData("Range in / bearing deg / roll deg", "%.1f / %.1f / %.1f",
                    cell.ftcPose.range, cell.ftcPose.bearing, cell.ftcPose.roll);
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Candidate", candidate);
            telemetry.addData("Agreement", "%d / %d", agreeingFrames, REQUIRED_FRAMES);
            telemetry.addData("Upward cell", confirmed);
            telemetry.addData("Vision target available", hasUpwardTarget());
            telemetry.addData("Selection reason", reason);
            for (AprilTagDetection d : detections) {
                if (!(d instanceof AprilTagClusterDetection)) continue;
                AprilTagClusterDetection c = (AprilTagClusterDetection) d;
                String rejected = rejectionReason(c);
                telemetry.addData("Filter " + cellOf(c),
                        rejected == null ? "PASS" : rejected);
            }
            }
    }
    private Cell cellOf(AprilTagClusterDetection c) {
        if (c.metadata == null || c.metadata.shortName == null)
            return Cell.UNKNOWN;
        String name = c.metadata.shortName.trim().toUpperCase(Locale.US);
        switch (name) {
            case "RED SCORING": return Cell.RED_SCORING;
            case "RED AUDIENCE": return Cell.RED_AUDIENCE;
            case "BLUE AUDIENCE": return Cell.BLUE_AUDIENCE;
            case "BLUE SCORING": return Cell.BLUE_SCORING;
            default: return Cell.UNKNOWN;
        }
    }
    public void setAlliance(Alliance value) {
        alliance = value == null ? Alliance.UNKNOWN : value;
        lose("Alliance changed; need new evidence");
    }
    private String rejectionReason(AprilTagClusterDetection c) {
        double ageMs = (System.nanoTime() - c.frameAcquisitionNanoTime) / 1e6;
        if (c.frameAcquisitionNanoTime <= 0 || ageMs < 0 || ageMs > MAX_AGE_MS)
            return "Stale or invalid timestamp";
        Cell id = cellOf(c);
        if (id == Cell.UNKNOWN) return "Unknown cluster name";
        if (alliance == Alliance.UNKNOWN) return "Select alliance";
        if (!id.name().startsWith(alliance.name() + "_"))
            return "Wrong alliance";
        if (c.ftcPose == null) return "No pose";
        double roll = c.ftcPose.roll;
        if (!Double.isFinite(roll) || !Double.isFinite(c.ftcPose.bearing)
                || !Double.isFinite(c.ftcPose.range) || c.ftcPose.range <= 0)
            return "Invalid pose";
        if (Math.abs(roll) >= 90.0) return "Orientation rejected";
        return null; // null means this cluster passed every filter
    }
// Helpers
    private void confirmFrame(long frameNanos) {
        if (candidate != pending) {
            pending = candidate;
            agreeingFrames = 0;
            lastFrameNanos = -1;
        }
        if (frameNanos > lastFrameNanos) {
            agreeingFrames = Math.min(REQUIRED_FRAMES, agreeingFrames + 1);
            lastFrameNanos = frameNanos;
        }
        confirmed = agreeingFrames >= REQUIRED_FRAMES ? candidate : Cell.UNKNOWN;
        reason = confirmed == Cell.UNKNOWN
                ? "Waiting for distinct frames" : "Upward target observed";
    }
    private void lose(String why) {
        candidate = confirmed = pending = Cell.UNKNOWN;
        agreeingFrames = 0;
        lastFrameNanos = -1;
        bearing = range = Double.NaN;
        reason = why;
    }
    public boolean hasUpwardTarget() { return confirmed != Cell.UNKNOWN; }
    public Cell getTargetCell() { return confirmed; }
    public double getBearingDegrees() {
        return hasUpwardTarget() ? bearing : Double.NaN;
    }
    public double getRangeInches() {
        return hasUpwardTarget() ? range : Double.NaN;
    }

    public void shutdown(){
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }
    }
