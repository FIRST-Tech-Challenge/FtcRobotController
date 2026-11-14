package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.List;

@Autonomous(name = "Limelight: AprilTag Auto Monitor", group = "Vision")
public class LimelightInfoReturn extends LinearOpMode {

    private static final double MAX_TAG_AGE_MS = 250;
    private static final double PIPELINE_SETTLE_SECONDS = 0.3;
    private static final double PIPELINE_RETRY_SECONDS = 1.0;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        if (limelight == null) {
            List<Limelight3A> cameras = hardwareMap.getAll(Limelight3A.class);
            if (cameras.size() == 1) {
                limelight = cameras.get(0);
            }
        }
        if (limelight == null) {
            telemetry.addLine("Configure one Limelight 3A, or name the intended camera "
                    + Constants.LIMELIGHT + ".");
            telemetry.addData("tv", 0);
            telemetry.update();
            return;
        }

        IMU imu = hardwareMap.tryGet(IMU.class, "imu");
        boolean imuReady = imu != null && imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        ElapsedTime pipelineTimer = new ElapsedTime();
        limelight.setPollRateHz(50);
        limelight.start();
        boolean pipelineSwitchSucceeded = limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        pipelineTimer.reset();
        telemetry.addLine("Press PLAY to monitor AprilTags; no drive or tilt-servo motion.");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                double imuYaw = Double.NaN;
                if (imuReady) {
                    YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                    imuYaw = angles.getYaw(AngleUnit.DEGREES);
                    limelight.updateRobotOrientation(imuYaw);
                }
                LLResult result = limelight.getLatestResult();
                boolean connected = limelight.isConnected();
                double resultAgeMs = result == null ? Double.NaN : result.getStaleness();
                List<LLResultTypes.FiducialResult> rawTags = result == null ? null : result.getFiducialResults();
                if (connected && pipelineTimer.seconds() >= PIPELINE_RETRY_SECONDS
                        && (!pipelineSwitchSucceeded || result == null
                        || result.getPipelineIndex() != Constants.LIMELIGHT_APRILTAG_PIPELINE)) {
                    pipelineSwitchSucceeded = limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
                    pipelineTimer.reset();
                }
                String resultStatus;
                if (!connected) {
                    resultStatus = "Disconnected";
                } else if (pipelineTimer.seconds() < PIPELINE_SETTLE_SECONDS) {
                    resultStatus = "Waiting for pipeline to settle";
                } else if (result == null) {
                    resultStatus = "No camera result received";
                } else if (result.getPipelineIndex() != Constants.LIMELIGHT_APRILTAG_PIPELINE) {
                    resultStatus = "Wrong pipeline returned";
                } else if (!(resultAgeMs >= 0 && resultAgeMs <= MAX_TAG_AGE_MS)) {
                    resultStatus = "Result is stale or has invalid age";
                } else if (rawTags == null || rawTags.isEmpty()) {
                    resultStatus = "No AprilTags detected in the current frame";
                } else {
                    resultStatus = "AprilTags detected";
                }
                // The fiducial list is the detection source; the primary-target flag is diagnostic only.
                boolean hasFreshAprilTagResult = "AprilTags detected".equals(resultStatus);
                LLResultTypes.FiducialResult target = null;
                int visibleTagCount = 0;

                if (hasFreshAprilTagResult) {
                    visibleTagCount = rawTags.size();
                    for (LLResultTypes.FiducialResult tag : rawTags) {
                        telemetry.addData("AprilTag " + tag.getFiducialId(),
                                "tx %.2f°, ty %.2f°", tag.getTargetXDegrees(), tag.getTargetYDegrees());
                        if (target == null || Math.abs(tag.getTargetXDegrees())
                                < Math.abs(target.getTargetXDegrees())) {
                            target = tag;
                        }
                    }
                }

                int tv = target == null ? 0 : 1;
                double tx = target == null ? Double.NaN : target.getTargetXDegrees();
                double ty = target == null ? Double.NaN : target.getTargetYDegrees();
                String action = "Monitoring only; no drive commands";

                telemetry.addData("Limelight", connected ? "Connected" : "Disconnected");
                telemetry.addData("Requested pipeline", Constants.LIMELIGHT_APRILTAG_PIPELINE + " (AprilTag)");
                telemetry.addData("Camera configuration names", hardwareMap.getNamesOf(limelight));
                telemetry.addData("Pipeline switch", pipelineSwitchSucceeded ? "Succeeded" : "Failed; retrying");
                telemetry.addLine("Configure pipeline " + Constants.LIMELIGHT_APRILTAG_PIPELINE
                        + " as AprilTag / 36h11 in the Limelight web UI.");
                telemetry.addData("Detection status", resultStatus);
                telemetry.addData("Actual pipeline", result == null ? "N/A" : result.getPipelineIndex());
                telemetry.addData("Camera result valid", result == null ? "N/A" : result.isValid());
                telemetry.addData("Result age (ms)", Double.isFinite(resultAgeMs)
                        ? String.format("%.1f", resultAgeMs) : "N/A");
                telemetry.addData("Raw AprilTags (before filtering)", sizeOf(rawTags));
                telemetry.addData("tv", tv);
                telemetry.addData("AprilTags detected", visibleTagCount);
                telemetry.addData("Target ID", target == null ? "None" : target.getFiducialId());
                telemetry.addData("tx (degrees)", Double.isFinite(tx) ? String.format("%.2f", tx) : "N/A");
                telemetry.addData("ty (degrees)", Double.isFinite(ty) ? String.format("%.2f", ty) : "N/A");
                telemetry.addData("Drive action", action);
                telemetry.addData("IMU yaw", Double.isFinite(imuYaw) ? String.format("%.1f°", imuYaw) : "Not configured");
                telemetry.addData("Image + processing latency (ms)", hasFreshAprilTagResult
                        ? String.format("%.1f", result.getCaptureLatency() + result.getTargetingLatency())
                        : "N/A");
                addPipelineDiagnostics(result);
                addMegaTagTelemetry(result, hasFreshAprilTagResult && result.isValid()
                        && Double.isFinite(imuYaw));
                telemetry.update();
                sleep(20);
            }
        } finally {
            limelight.stop();
        }
    }

    /** Reports pipeline contents separately from fresh AprilTag detections. */
    private void addPipelineDiagnostics(LLResult result) {
        if (result == null) {
            telemetry.addData("Pipeline type (reported)", "N/A");
            telemetry.addData("Fiducial / Color / Detector / Barcode", "N/A");
            return;
        }

        int fiducialCount = sizeOf(result.getFiducialResults());
        int colorCount = sizeOf(result.getColorResults());
        int detectorCount = sizeOf(result.getDetectorResults());
        int barcodeCount = sizeOf(result.getBarcodeResults());

        telemetry.addData("Pipeline type (reported)", result.getPipelineType());
        telemetry.addData("Fiducial / Color / Detector / Barcode", "%d / %d / %d / %d",
                fiducialCount, colorCount, detectorCount, barcodeCount);

        if (fiducialCount == 0 && (colorCount + detectorCount + barcodeCount) > 0) {
            telemetry.addLine("WARNING: pipeline is returning non-AprilTag results. "
                    + "Set it to Fiducial Markers (36h11) in the Limelight web UI, "
                    + "or point Constants.LIMELIGHT_APRILTAG_PIPELINE at the right slot.");
        } else if (fiducialCount == 0) {
            telemetry.addLine("0 fiducials returned. Check the pipeline type is Fiducial Markers, "
                    + "the family is 36h11, the ID filter allows your tags, and the web UI stream "
                    + "draws a box around the tag.");
        }
    }

    private static int sizeOf(List<?> list) {
        return list == null ? 0 : list.size();
    }

    private void addMegaTagTelemetry(LLResult result, boolean hasFreshAprilTagResult) {
        telemetry.addLine("MegaTag2 needs a loaded field map and configured camera-to-robot pose.");
        if (!hasFreshAprilTagResult || result == null || result.getBotposeTagCount() < 1) {
            telemetry.addData("MegaTag2 botpose", "Unavailable");
            return;
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null || botpose.getPosition() == null || botpose.getOrientation() == null) {
            telemetry.addData("MegaTag2 botpose", "Unavailable");
            return;
        }

        Position inches = botpose.getPosition().toUnit(DistanceUnit.INCH);
        telemetry.addData("MegaTag2 X / Y / Z (in)", "%.1f / %.1f / %.1f",
                inches.x, inches.y, inches.z);
        telemetry.addData("MegaTag2 yaw (degrees)", "%.1f",
                botpose.getOrientation().getYaw(AngleUnit.DEGREES));
        telemetry.addData("MegaTag2 tags used", result.getBotposeTagCount());
    }
}