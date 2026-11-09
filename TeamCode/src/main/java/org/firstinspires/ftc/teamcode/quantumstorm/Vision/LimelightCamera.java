package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.List;

/**
 * Limelight 3A on a tilt servo. One pipeline runs at a time:
 *  - false / tiltDown(): color pipeline, used to find game pieces
 *  - true / tiltUp(): AprilTag pipeline, used to find the Hive
 *
 * If the camera or servo is missing from the Robot Configuration, this does not
 * crash INIT: isConnected() returns false and every query reports "nothing seen",
 * so the autonomous self-check can send the robot to ERROR -> PARK instead.
 */
public class LimelightCamera {

    private static final long MAX_TELEMETRY_RESULT_AGE_MS = 250;

    private final Limelight3A limelight;   // null if not in the configuration
    private final Servo tiltServo;         // null if not in the configuration
    private boolean aprilTagMode = false;  // Start in color-search mode.

    public LimelightCamera(HardwareMap hardwareMap) {
        limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        tiltServo = hardwareMap.tryGet(Servo.class, Constants.LIMELIGHT_TILT_SERVO);
        // Initialize in false/color mode without toggling the mode flag.
        if (tiltServo != null) {
            tiltServo.setPosition(Constants.LIMELIGHT_TILT_DOWN);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(Constants.LIMELIGHT_COLOR_PIPELINE);
            limelight.start();
        }
    }

    public boolean isConnected() {
        return limelight != null && tiltServo != null && limelight.isConnected();
    }

    public void tiltUp() {
        setMode(true);
    }

    public void tiltDown() {
        setMode(false);
    }

    /** Toggle modes whenever the Limelight tilt servo should move. */
    public void toggleMode() {
        setMode(!aprilTagMode);
    }

    /** True means AprilTag mode; false means color-search mode. */
    public boolean isAprilTagMode() {
        return aprilTagMode;
    }

    // ---- AprilTag pipeline ----

    /** True if the AprilTag pipeline currently sees at least one tag. */
    public boolean isAprilTagVisible() {
        LLResult result = resultFor(Constants.LIMELIGHT_APRILTAG_PIPELINE);
        return result != null && !result.getFiducialResults().isEmpty();
    }

    /** Horizontal angle to the seen AprilTag in degrees (right = positive), or 0 if none. */
    public double getAprilTagAngle() {
        LLResult result = resultFor(Constants.LIMELIGHT_APRILTAG_PIPELINE);
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
        LLResult result = resultFor(Constants.LIMELIGHT_COLOR_PIPELINE);
        return result != null ? result.getTx() : 0;
    }

    /** True when the color pipeline reports an actual color or detector target. */
    public boolean isColorVisible() {
        LLResult result = resultFor(Constants.LIMELIGHT_COLOR_PIPELINE);
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
        LLResult result = resultFor(Constants.LIMELIGHT_COLOR_PIPELINE);
        return result != null && isColorVisible() ? result.getTx() : 0;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // ---- helpers ----

    private void setMode(boolean useAprilTags) {
        if (aprilTagMode == useAprilTags) {
            return; // No servo movement, so do not toggle the mode.
        }

        aprilTagMode = useAprilTags;
        double tilt = useAprilTags ? Constants.LIMELIGHT_TILT_UP : Constants.LIMELIGHT_TILT_DOWN;
        int pipeline = useAprilTags
                ? Constants.LIMELIGHT_APRILTAG_PIPELINE
                : Constants.LIMELIGHT_COLOR_PIPELINE;

        if (tiltServo != null) {
            tiltServo.setPosition(tilt);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
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
