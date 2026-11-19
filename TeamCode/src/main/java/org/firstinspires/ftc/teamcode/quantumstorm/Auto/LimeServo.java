package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

public class LimeServo {

    private static final double APRIL_TAG_TILT_FROM_NEUTRAL_DEGREES = 0.0;
    private static final double COLOR_TILT_FROM_NEUTRAL_DEGREES = 120.0;

    public enum Pipeline {
        APRIL_TAG(Constants.LIMELIGHT_APRILTAG_PIPELINE),
        BLUE(Constants.LIMELIGHT_BLUE_PIPELINE),
        YELLOW(Constants.LIMELIGHT_YELLOW_PIPELINE),
        RED(Constants.LIMELIGHT_RED_PIPELINE);

        private final int index;

        Pipeline(int index) {
            this.index = index;
        }
    }

    private final Limelight3A limelight;
    private final Servo tiltServo;
    private final DoubleSupplier clock;
    private Pipeline activePipeline = Pipeline.APRIL_TAG;
    private boolean scanning;
    private boolean pickupScanning;
    private double switchedAt;
    private double commandedTiltDegrees;

    public LimeServo(HardwareMap hardwareMap) {
        this(hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT),
                hardwareMap.tryGet(Servo.class, Constants.LIMELIGHT_TILT_SERVO),
                () -> System.nanoTime() / 1e9);
        if (limelight != null) {
            limelight.start();
        }
    }

    /** Share the autonomous camera instead of starting a second camera controller. */
    public LimeServo(Limelight3A limelight, Servo tiltServo, DoubleSupplier clock) {
        this.limelight = limelight;
        this.tiltServo = tiltServo;
        this.clock = clock;
        switchedAt = clock.getAsDouble();

        if (tiltServo != null) {
            tiltServo.setPosition(Constants.LIMELIGHT_SERVO_NEUTRAL_POSITION);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(activePipeline.index);
        }
    }

    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    public void selectPipeline(Pipeline pipeline) {
        if (pipeline == null) {
            throw new IllegalArgumentException("Pipeline is required");
        }
        scanning = false;
        pickupScanning = false;
        boolean pipelineChanged = pipeline != activePipeline;
        activePipeline = pipeline;
        setTiltForActivePipeline();
        if (pipelineChanged && limelight != null) {
            limelight.pipelineSwitch(pipeline.index);
        }
        if (pipelineChanged) {
            switchedAt = clock.getAsDouble();
        }
    }

    /** Call each loop: shooting uses neutral/AprilTags; idle scans red and blue. */
    public void update(boolean shooting) {
        if (shooting) {
            selectPipeline(Pipeline.APRIL_TAG);
        } else if (!scanning) {
            selectPipeline(Pipeline.RED);
            switchedAt = clock.getAsDouble();
            scanning = true;
        } else if (clock.getAsDouble() - switchedAt >= Constants.LIMELIGHT_COLOR_PIPELINE_DWELL_SEC) {
            selectPipeline(activePipeline == Pipeline.RED ? Pipeline.BLUE : Pipeline.RED);
            scanning = true;
        }

    }

    /** Pickup cycles all color slots without moving the camera away from +120 degrees. */
    public void updatePickup() {
        if (!pickupScanning) {
            selectPipeline(Pipeline.YELLOW);
            switchedAt = clock.getAsDouble();
            pickupScanning = true;
        } else if (clock.getAsDouble() - switchedAt >= Constants.LIMELIGHT_COLOR_PIPELINE_DWELL_SEC) {
            Pipeline next = activePipeline == Pipeline.YELLOW ? Pipeline.BLUE
                    : activePipeline == Pipeline.BLUE ? Pipeline.RED : Pipeline.YELLOW;
            selectPipeline(next);
            pickupScanning = true;
        }
    }

    public int getPipelineIndex() {
        return activePipeline.index;
    }

    public LLResult getFreshResult() {
        if (clock.getAsDouble() - switchedAt < 0.20) {
            return null;
        }
        LLResult result = getLatestResult();
        return isFreshResult(result) ? result : null;
    }

    public LLResult getLatestResult() {
        return isConnected() ? limelight.getLatestResult() : null;
    }

    public boolean isFreshResult(LLResult result) {
        return clock.getAsDouble() - switchedAt >= 0.20
                && result != null && result.isValid() && result.getPipelineIndex() == activePipeline.index
                && result.getStaleness() >= 0 && result.getStaleness() <= 250;
    }

    public double getCommandedTiltDegrees() {
        return tiltServo == null ? Double.NaN : commandedTiltDegrees;
    }

    public double getCommandedTiltPosition() {
        return tiltServo == null ? Double.NaN : Constants.LIMELIGHT_SERVO_NEUTRAL_POSITION
                + commandedTiltDegrees / Constants.LIMELIGHT_SERVO_TRAVEL_DEGREES;
    }

    public double getSecondsUntilScanSwitch() {
        return scanning ? Math.max(0, Constants.LIMELIGHT_COLOR_PIPELINE_DWELL_SEC
                - (clock.getAsDouble() - switchedAt)) : Double.NaN;
    }

    public Pipeline getActivePipeline() {
        return activePipeline;
    }

    public double getHorizontalAngle() {
        LLResult result = getFreshResult();
        return result == null ? 0 : result.getTx();
    }

    public List<Integer> getVisibleAprilTagIds() {
        if (limelight == null || activePipeline != Pipeline.APRIL_TAG) {
            return Collections.emptyList();
        }
        LLResult result = getFreshResult();
        if (result == null) {
            return Collections.emptyList();
        }

        List<LLResultTypes.FiducialResult> detections = result.getFiducialResults();
        List<Integer> ids = new ArrayList<>();
        if (detections != null) {
            for (LLResultTypes.FiducialResult detection : detections) {
                ids.add(detection.getFiducialId());
            }
        }
        return ids;
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
            commandedTiltDegrees = limitedAngle;
        }
    }

    private void setTiltForActivePipeline() {
        double angle = activePipeline == Pipeline.APRIL_TAG
                ? APRIL_TAG_TILT_FROM_NEUTRAL_DEGREES
                : COLOR_TILT_FROM_NEUTRAL_DEGREES;
        setTiltAngleFromNeutralDegrees(angle);
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
