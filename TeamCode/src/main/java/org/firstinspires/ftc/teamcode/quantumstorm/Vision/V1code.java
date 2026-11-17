package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.List;

/**
 * Reports firing permission only; never drives or runs the shooter.
 * Configure Limelight pipeline 1 for 36h11 with the correct physical tag size.
 * D-pad left selects red; right selects blue. With the camera mounted 30 degrees
 * upward, hold a raised tag directly facing it and press A to calibrate orientation.
 * For the first 10 seconds after Start, require calibration, the selected alliance
 * ID, and orientation within 5 degrees. Then any fresh visible AprilTag permits
 * firing, regardless of ID, orientation, calibration, or number of visible tags.
 * Fallback freshness uses Control Hub result age, not the camera-local timestamp.
 * FIRE PERMITTED is telemetry only: this OpMode does not activate the shooter.
 * No visible tag means STOPPED; permission is not latched when a tag disappears.
 * Calibration uses camera-space pose; do not add another 30-degree correction.
 */
@Disabled
@TeleOp(name = "V1 AprilTag Fire Permission", group = "Vision")
public class V1code extends LinearOpMode {
    private static final String LIMELIGHT_NAME = "limelight 3a";
    private static final int APRILTAG_PIPELINE = Constants.LIMELIGHT_APRILTAG_PIPELINE;
    private static final long MAX_RESULT_AGE_MS = 250;

    public boolean isBlue = false;
    public boolean availableToFire = false;
    public boolean blueFire = false;
    public boolean redFire = false;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);
        if (limelight == null) {
            telemetry.addData("Limelight", "Configure device name '%s'.", LIMELIGHT_NAME);
            telemetry.update();
            return;
        }

        AprilTagFireGate gate = new AprilTagFireGate();
        boolean previousA = false;
        long startTimeNanos = -1;
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        try {
            limelight.start();
            while (!isStopRequested() && (opModeInInit() || opModeIsActive())) {
                boolean active = opModeIsActive();
                long now = System.nanoTime();
                if (active && startTimeNanos == -1) {
                    startTimeNanos = now;
                }
                double elapsedSeconds = active ? (now - startTimeNanos) / 1e9 : 0;
                boolean strictChecks = elapsedSeconds < AprilTagFireGate.STRICT_STARTUP_SECONDS;
                boolean selectedBlue = gamepad1.dpad_right || (isBlue && !gamepad1.dpad_left);
                if (selectedBlue != isBlue) {
                    isBlue = selectedBlue;
                    gate.clearCalibration();
                }
                boolean calibrate = gamepad1.a && !previousA;
                previousA = gamepad1.a;
                LLResult result = limelight.getLatestResult();
                LLResultTypes.FiducialResult target = null;
                int visibleTagCount = 0;
                boolean connected = limelight.isConnected();
                long resultAgeMs = result == null ? -1 : result.getStaleness();
                boolean fresh = connected && result != null && result.isValid()
                        && result.getPipelineIndex() == APRILTAG_PIPELINE
                        && resultAgeMs >= 0 && resultAgeMs <= MAX_RESULT_AGE_MS;
                if (fresh) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    visibleTagCount = tags == null ? 0 : tags.size();
                    // Strict mode requires one target; fallback accepts any visible tags.
                    if (visibleTagCount == 1 || (!strictChecks && visibleTagCount > 0)) {
                        target = tags.get(0);
                    }
                }

                int tagId = target == null ? -1 : target.getFiducialId();
                Pose3D pose = target == null ? null : target.getTargetPoseCameraSpace();
                double yaw = Double.NaN;
                double pitch = Double.NaN;
                double roll = Double.NaN;
                if (pose != null && pose.getPosition() != null
                        && Double.isFinite(pose.getPosition().x)
                        && Double.isFinite(pose.getPosition().y)
                        && Double.isFinite(pose.getPosition().z)
                        && pose.getPosition().z > 0 && pose.getOrientation() != null) {
                    // The SDK substitutes a zero pose when pose data is missing.
                    YawPitchRollAngles orientation = pose.getOrientation();
                    yaw = orientation.getYaw(AngleUnit.DEGREES);
                    pitch = orientation.getPitch(AngleUnit.DEGREES);
                    roll = orientation.getRoll(AngleUnit.DEGREES);
                }

                if (strictChecks && calibrate) {
                    gate.clearCalibration();
                    if (AprilTagFireGate.matchesAlliance(tagId, isBlue)) {
                        gate.calibrate(yaw, pitch, roll);
                    }
                }
                boolean permission = gate.updateDuringRun(tagId, isBlue, yaw, pitch, roll,
                        result == null ? Double.NaN : result.getTimestamp(), fresh,
                        visibleTagCount > 0, elapsedSeconds);
                availableToFire = opModeIsActive() && (!strictChecks || !gamepad1.a) && permission;
                blueFire = isBlue && availableToFire;
                redFire = !isBlue && availableToFire;

                telemetry.addLine("D-pad left: RED | right: BLUE");
                telemetry.addLine(strictChecks
                        ? "Face a raised tag directly at camera; press A to calibrate."
                        : "Any fresh visible AprilTag permits firing; no angle/ID checks.");
                telemetry.addData("Mode", strictChecks ? "STRICT STARTUP" : "ANY TAG");
                telemetry.addData("Seconds after Start", "%.1f", elapsedSeconds);
                telemetry.addData("Visible tags", visibleTagCount);
                telemetry.addData("Camera connected", connected);
                telemetry.addData("Result valid", result != null && result.isValid());
                telemetry.addData("Pipeline (expected 1)", result == null ? -1 : result.getPipelineIndex());
                telemetry.addData("Result age (ms; max 250)", resultAgeMs);
                telemetry.addData("Fresh result", fresh);
                telemetry.addData("isBlue", isBlue);
                telemetry.addData("Calibrated", gate.isCalibrated());
                telemetry.addData("Tag ID", tagId);
                telemetry.addData("Yaw / pitch / roll", "%.1f / %.1f / %.1f", yaw, pitch, roll);
                telemetry.addData("Orientation error (degrees)", gate.getOrientationError());
                telemetry.addData("availableToFire", availableToFire);
                telemetry.addData("blueFire", blueFire);
                telemetry.addData("redFire", redFire);
                telemetry.addData("Decision", availableToFire
                        ? (strictChecks ? (isBlue ? "Blue fire" : "Available to fire (red)")
                                : "FIRE PERMITTED (ANY TAG)")
                        : (visibleTagCount == 0 ? "STOPPED" : "DO NOT FIRE"));
                telemetry.addData("Permission blocker", availableToFire ? "NONE"
                        : !opModeIsActive() ? "Press START"
                        : !fresh ? "Camera result missing, invalid, wrong pipeline, or stale"
                        : visibleTagCount == 0 ? "No AprilTag visible"
                        : strictChecks ? "Startup calibration, alignment, or stability check"
                        : "Fallback permission unavailable");
                telemetry.addLine("Permission only; shooter is NOT activated.");
                telemetry.update();
                sleep(20);
            }
        } finally {
            availableToFire = false;
            blueFire = false;
            redFire = false;
            limelight.stop();
        }
    }
}
