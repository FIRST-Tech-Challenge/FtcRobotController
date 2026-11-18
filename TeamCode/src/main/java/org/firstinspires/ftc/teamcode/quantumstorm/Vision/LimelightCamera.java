package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.List;

/**
 * Limelight 3A and tilt servo. AprilTags are used outside INTAKE; INTAKE cycles
 * the yellow, blue, and red color pipelines for telemetry only.
 *
 * If the camera or servo is missing from the Robot Configuration, this does not
 * crash INIT: isConnected() returns false, so the autonomous self-check sends
 * the robot to ERROR -> PARK. A connected camera can still report detections
 * when the tilt servo is missing; detections do not imply hardware readiness.
 */
public class LimelightCamera {

    private static final long MAX_TELEMETRY_RESULT_AGE_MS = 250;
    private static final int[] COLOR_PIPELINES = {
            Constants.LIMELIGHT_YELLOW_PIPELINE,
            Constants.LIMELIGHT_BLUE_PIPELINE,
            Constants.LIMELIGHT_RED_PIPELINE
    };
    private static final String[] COLOR_PIPELINE_NAMES = {"Yellow", "Blue", "Red"};

    private final Limelight3A limelight;   // null if not in the configuration
    private final Servo tiltServo;         // null if not in the configuration
    private final ElapsedTime colorPipelineTimer = new ElapsedTime();
    private boolean aprilTagMode = true;
    private boolean colorPipelineScanActive;
    private int activePipeline = Constants.LIMELIGHT_APRILTAG_PIPELINE;
    private int colorPipelineScanIndex;

    public LimelightCamera(HardwareMap hardwareMap) {
        limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        tiltServo = hardwareMap.tryGet(Servo.class, Constants.LIMELIGHT_TILT_SERVO);
        // Start neutral and use AprilTags until autonomous enters INTAKE.
        if (tiltServo != null) {
            tiltServo.setPosition(Constants.LIMELIGHT_SERVO_NEUTRAL_POSITION);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(Constants.LIMELIGHT_APRILTAG_PIPELINE);
            limelight.start();
        }
    }

    public boolean isConnected() {
        return limelight != null && tiltServo != null && limelight.isConnected();
    }

    public String getConnectionStatus() {
        if (limelight == null) {
            return "Limelight not configured: " + Constants.LIMELIGHT;
        }
        if (tiltServo == null) {
            return "Tilt servo not configured: " + Constants.LIMELIGHT_TILT_SERVO;
        }
        return limelight.isConnected() ? "Ready" : "Limelight disconnected: " + Constants.LIMELIGHT;
    }

    public void selectAprilTagPipeline() {
        setMode(true);
    }

    public void selectColorPipeline() {
        setMode(false);
    }

    public void startColorPipelineScan() {
        aprilTagMode = false;
        colorPipelineScanActive = true;
        colorPipelineScanIndex = 0;
        colorPipelineTimer.reset();
        switchPipeline(COLOR_PIPELINES[colorPipelineScanIndex]);
    }

    public void updateColorPipelineScan() {
        if (!colorPipelineScanActive
                || colorPipelineTimer.seconds() < Constants.LIMELIGHT_COLOR_PIPELINE_DWELL_SEC) {
            return;
        }
        colorPipelineScanIndex = (colorPipelineScanIndex + 1) % COLOR_PIPELINES.length;
        switchPipeline(COLOR_PIPELINES[colorPipelineScanIndex]);
        colorPipelineTimer.reset();
    }

    /** Toggle between color and AprilTag pipelines. */
    public void toggleMode() {
        setMode(!aprilTagMode);
    }

    /** True means AprilTag mode; false means color-search mode. */
    public boolean isAprilTagMode() {
        return aprilTagMode;
    }

    /** Positive angles command clockwise motion; reverse servo direction if the mounting is opposite. */
    public void setTiltAngleFromNeutralDegrees(double clockwisePositiveDegrees) {
        if (!Double.isFinite(clockwisePositiveDegrees)) {
            throw new IllegalArgumentException("Tilt angle must be finite");
        }
        if (tiltServo != null) {
            double limitedAngle = Range.clip(clockwisePositiveDegrees,
                    -Constants.LIMELIGHT_SERVO_MAX_OFFSET_DEGREES,
                    Constants.LIMELIGHT_SERVO_MAX_OFFSET_DEGREES);
            double servoPosition = Constants.LIMELIGHT_SERVO_NEUTRAL_POSITION
                    + limitedAngle / Constants.LIMELIGHT_SERVO_TRAVEL_DEGREES;
            tiltServo.setPosition(servoPosition);
        }
    }

    // ---- AprilTag pipeline ----

    /** Requires camera tv=1 (SDK isValid), a fresh frame, and an AprilTag detection. */
    public boolean isAprilTagVisible() {
        LLResult result = telemetryResultFor(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        return result != null && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty();
    }

    /** Horizontal angle to the seen AprilTag in degrees (right = positive), or 0 if none. */
    public double getAprilTagAngle() {
        LLResult result = telemetryResultFor(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        return result != null ? result.getTx() : 0;
    }

    /**
     * True once the Hive's active Cell has tipped to the far face.
     * TODO: decide from the visible Hive AprilTag ID(s) once the tag-to-face
     *       mapping is confirmed in the game manual (design doc: IsHiveFlipped).
     */
    public boolean isHiveFlipped() {
        return false;
    }

    // ---- Ball pipeline ----

    /** True if the ball pipeline currently sees a ball. */
    public boolean isBallVisible() {
        return isColorVisible();
    }

    /** Horizontal angle to the seen ball in degrees (right = positive), or 0 if none. */
    public double getBallAngle() {
        LLResult result = currentColorResult();
        return result != null ? result.getTx() : 0;
    }

    /** True when the color pipeline reports an actual color or detector target. */
    public boolean isColorVisible() {
        LLResult result = currentColorResult();
        if (result == null) {
            return false;
        }

        List<LLResultTypes.ColorResult> colors = result.getColorResults();
        List<LLResultTypes.DetectorResult> objects = result.getDetectorResults();
        return (colors != null && !colors.isEmpty())
                || (objects != null && !objects.isEmpty());
    }

    /** Horizontal angle to the target in the color pipeline, or 0 if none. */
    public double getColorAngle() {
        LLResult result = currentColorResult();
        return result != null && isColorVisible() ? result.getTx() : 0;
    }

    /** Supply robot heading to the Limelight's MegaTag2 localization. */
    public void updateRobotOrientation(double yawDegrees) {
        if (limelight != null && Double.isFinite(yawDegrees)) {
            limelight.updateRobotOrientation(yawDegrees);
        }
    }

    /** Add the current pipeline's target and pose data to Driver Station telemetry. */
    public void addTelemetry(Telemetry telemetry) {
        boolean connected = limelight != null && limelight.isConnected();
        int pipeline = activePipeline;
        telemetry.addData("Limelight", connected ? "Connected" : "Disconnected / not found");
        telemetry.addData("Camera hardware", getConnectionStatus());
        telemetry.addData("Limelight pipeline", "%d (%s)", pipeline, pipelineName(pipeline));
        if (colorPipelineScanActive) {
            telemetry.addData("Color pipeline scan", "next switch in %.1f s",
                    Math.max(0.0, Constants.LIMELIGHT_COLOR_PIPELINE_DWELL_SEC
                            - colorPipelineTimer.seconds()));
        }

        LLResult result = telemetryResultFor(pipeline);
        boolean targetFound = false;
        int targetCount = 0;
        String targetId = "None";
        double tx = Double.NaN;
        double ty = Double.NaN;
        double targetArea = Double.NaN;

        if (result != null && aprilTagMode) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null) {
                targetCount = tags.size();
                LLResultTypes.FiducialResult best = null;
                for (LLResultTypes.FiducialResult tag : tags) {
                    if (best == null || Math.abs(tag.getTargetXDegrees())
                            < Math.abs(best.getTargetXDegrees())) {
                        best = tag;
                    }
                }
                if (best != null) {
                    targetFound = true;
                    targetId = Integer.toString(best.getFiducialId());
                    tx = best.getTargetXDegrees();
                    ty = best.getTargetYDegrees();
                }
            }
        } else if (result != null) {
            List<LLResultTypes.ColorResult> colors = result.getColorResults();
            if (colors != null) {
                targetCount += colors.size();
                for (LLResultTypes.ColorResult color : colors) {
                    if (color.getTargetArea() > targetArea) {
                        targetFound = true;
                        targetArea = color.getTargetArea();
                        tx = color.getTargetXDegrees();
                        ty = color.getTargetYDegrees();
                        targetId = "Color target";
                    }
                }
            }
            List<LLResultTypes.DetectorResult> objects = result.getDetectorResults();
            if (objects != null) {
                targetCount += objects.size();
                for (LLResultTypes.DetectorResult object : objects) {
                    if (object.getTargetArea() > targetArea) {
                        targetFound = true;
                        targetArea = object.getTargetArea();
                        tx = object.getTargetXDegrees();
                        ty = object.getTargetYDegrees();
                        targetId = object.getClassName();
                    }
                }
            }
        }

        telemetry.addData("tv", targetFound ? 1 : 0);
        telemetry.addData("Targets detected", targetCount);
        telemetry.addData("Target ID / class", targetId);
        telemetry.addData("tx / ty (degrees)", targetFound
                ? String.format(java.util.Locale.US, "%.2f / %.2f", tx, ty) : "N/A");
        if (Double.isFinite(targetArea)) {
            telemetry.addData("Target area", "%.2f%%", targetArea);
        }
        if (result == null) {
            telemetry.addData("Limelight latency (ms)", "N/A");
        } else {
            telemetry.addData("Limelight latency (ms)", "%.1f",
                    result.getCaptureLatency() + result.getTargetingLatency());
        }

        if (aprilTagMode) {
            addMegaTagTelemetry(telemetry, result);
        }
    }

    private LLResult telemetryResultFor(int pipeline) {
        if (limelight == null || !limelight.isConnected()) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && result.getPipelineIndex() == pipeline
                && result.getStaleness() >= 0 && result.getStaleness() <= MAX_TELEMETRY_RESULT_AGE_MS
                ? result : null;
    }

    private void addMegaTagTelemetry(Telemetry telemetry, LLResult result) {
        if (result == null || result.getBotposeTagCount() < 1) {
            telemetry.addData("MegaTag2 botpose", "Unavailable");
            telemetry.addLine("MegaTag2 requires a field map and configured camera pose.");
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

    public void stop() {
        colorPipelineScanActive = false;
        if (limelight != null) {
            limelight.stop();
        }
    }

    // ---- helpers ----

    private void setMode(boolean useAprilTags) {
        colorPipelineScanActive = false;
        aprilTagMode = useAprilTags;
        int pipeline = useAprilTags
                ? Constants.LIMELIGHT_APRILTAG_PIPELINE
                : Constants.LIMELIGHT_COLOR_PIPELINE;
        switchPipeline(pipeline);
    }

    private void switchPipeline(int pipeline) {
        if (activePipeline == pipeline) {
            return;
        }
        activePipeline = pipeline;
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    private LLResult currentColorResult() {
        return aprilTagMode ? null : resultFor(activePipeline);
    }

    private String pipelineName(int pipeline) {
        if (pipeline == Constants.LIMELIGHT_APRILTAG_PIPELINE) {
            return "AprilTag";
        }
        for (int index = 0; index < COLOR_PIPELINES.length; index++) {
            if (pipeline == COLOR_PIPELINES[index]) {
                return COLOR_PIPELINE_NAMES[index];
            }
        }
        return "Unknown";
    }

    /** Latest valid result from the given pipeline, or null after a pipeline switch. */
    private LLResult resultFor(int pipeline) {
        if (limelight == null) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != pipeline) {
            return null;
        }
        return result;
    }
}
