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

public class LimeServo {

    private static final double APRIL_TAG_TILT_FROM_NEUTRAL_DEGREES = -100.0;
    private static final double COLOR_TILT_FROM_NEUTRAL_DEGREES = 60.0;

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
    private Pipeline activePipeline = Pipeline.BLUE;

    public LimeServo(HardwareMap hardwareMap) {
        limelight = hardwareMap.tryGet(Limelight3A.class, Constants.LIMELIGHT);
        tiltServo = hardwareMap.tryGet(Servo.class, Constants.LIMELIGHT_TILT_SERVO);

        if (tiltServo != null) {
            tiltServo.setPosition(Constants.LIMELIGHT_SERVO_NEUTRAL_POSITION);
        }
        if (limelight != null) {
            limelight.pipelineSwitch(activePipeline.index);
            limelight.start();
        }
    }

    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    public void selectPipeline(Pipeline pipeline) {
        if (pipeline == null) {
            return;
        }
        boolean pipelineChanged = pipeline != activePipeline;
        activePipeline = pipeline;
        setTiltForActivePipeline();
        if (pipelineChanged && limelight != null) {
            limelight.pipelineSwitch(pipeline.index);
        }
    }

    public Pipeline getActivePipeline() {
        return activePipeline;
    }

    public double getHorizontalAngle() {
        if (limelight == null) {
            return 0;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != activePipeline.index) {
            return 0;
        }
        return result.getTx();
    }

    public List<Integer> getVisibleAprilTagIds() {
        if (limelight == null || activePipeline != Pipeline.APRIL_TAG) {
            return Collections.emptyList();
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != activePipeline.index) {
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
