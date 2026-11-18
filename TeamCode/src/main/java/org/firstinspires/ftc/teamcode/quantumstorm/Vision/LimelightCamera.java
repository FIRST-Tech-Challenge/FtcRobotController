package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/**
 * Limelight 3A on a tilt servo. One pipeline runs at a time:
 *  - tiltUp():   AprilTag pipeline, used before shooting at the Hive
 *  - tiltDown(): ball pipeline, used to find balls on the floor
 *
 * If the camera or servo is missing from the Robot Configuration, this does not
 * crash INIT: isConnected() returns false and every query reports "nothing seen",
 * so the autonomous self-check can send the robot to ERROR -> PARK instead.
 */
public class LimelightCamera {

    private final Limelight3A limelight;   // null if not in the configuration
    private final Servo tiltServo;         // null if not in the configuration

    public LimelightCamera(HardwareMap hardwareMap) {
        limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        tiltServo = hardwareMap.tryGet(Servo.class, Constants.LIMELIGHT_TILT_SERVO);
        if (limelight != null) {
            limelight.start();
        }
        tiltUp();
    }

    public boolean isConnected() {
        return limelight != null && tiltServo != null && limelight.isConnected();
    }

    public void tiltUp() {
        setTiltAndPipeline(Constants.LIMELIGHT_TILT_UP, Constants.LIMELIGHT_APRILTAG_PIPELINE);
    }

    public void tiltDown() {
        setTiltAndPipeline(Constants.LIMELIGHT_TILT_DOWN, Constants.LIMELIGHT_BALL_PIPELINE);
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
        return resultFor(Constants.LIMELIGHT_BALL_PIPELINE) != null;
    }

    /** Horizontal angle to the seen ball in degrees (right = positive), or 0 if none. */
    public double getBallAngle() {
        LLResult result = resultFor(Constants.LIMELIGHT_BALL_PIPELINE);
        return result != null ? result.getTx() : 0;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // ---- helpers ----

    private void setTiltAndPipeline(double tilt, int pipeline) {
        if (tiltServo != null) {
            tiltServo.setPosition(tilt);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }

    /** Latest valid result from the given pipeline, or null (right after a switch results are from the old one). */
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