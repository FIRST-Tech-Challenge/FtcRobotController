package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

import java.util.List;

/** Reports every AprilTag detected by the Limelight's 36h11 pipeline. */
@Disabled
@Autonomous(name = "Search AprilTags (36h11)", group = "Vision")
public class SearchAprilTag extends LinearOpMode {

    private static final String LIMELIGHT_NAME = Constants.LIMELIGHT;
    private static final int APRILTAG_PIPELINE = Constants.LIMELIGHT_APRILTAG_PIPELINE;
    private static final int REQUIRED_STABLE_FRAMES = 3;
    private static final long MAX_RESULT_AGE_MS = 250;
    private static final String SHOOT_ALLOWED = "SHOOT ALLOWED";
    private static final String HOLD = "HOLD";
    private static final String UNCERTAIN = "NO TAG / UNCERTAIN";

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT_NAME);
        if (limelight == null) {
            telemetry.addData("Limelight", "Not found; check Robot Configuration name '%s'.",
                    LIMELIGHT_NAME);
            telemetry.update();
            return;
        }

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        telemetry.addLine("Pipeline 1 must be configured for AprilTag family 36h11.");
        telemetry.addLine("Press Start to report visible tag IDs.");
        telemetry.update();
        waitForStart();

        String lastCandidate = UNCERTAIN;
        int stableFrames = 0;
        double lastTimestamp = Double.NaN;

        try {
            while (opModeIsActive()) {
                String frameCandidate = UNCERTAIN;
                double frameTimestamp = Double.NaN;
                telemetry.addData("Limelight connection", limelight.isConnected()
                        ? "Connected"
                        : "Not connected");
                LLResult result = limelight.getLatestResult();

                if (!limelight.isConnected()) {
                    telemetry.addLine("Check the Limelight power, network, and Robot Configuration.");
                } else if (result == null) {
                    telemetry.addLine("Waiting for camera results.");
                } else {
                    int activePipeline = result.getPipelineIndex();
                    telemetry.addData("Pipeline", activePipeline);
                    telemetry.addData("Result valid", result.isValid());

                    if (activePipeline != APRILTAG_PIPELINE) {
                        telemetry.addLine("Waiting for pipeline 1; set it to AprilTag 36h11.");
                    } else if (!result.isValid()) {
                        telemetry.addLine("No valid camera result yet.");
                    } else {
                        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                        if (tags == null || tags.isEmpty()) {
                            telemetry.addLine("No tag / uncertain.");
                        } else {
                            boolean hasShootTag = false;
                            boolean hasHoldTag = false;
                            boolean hasUnknownTag = false;
                            telemetry.addData("36h11 AprilTags", tags.size());
                            for (LLResultTypes.FiducialResult tag : tags) {
                                int tagId = tag.getFiducialId();
                                telemetry.addData("Tag ID " + tagId,
                                        "%s | horizontal %.1f° | vertical %.1f°",
                                        getTagClassification(tagId),
                                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
                                if (isShootTag(tagId)) {
                                    hasShootTag = true;
                                } else if (isKnownTag(tagId)) {
                                    hasHoldTag = true;
                                } else {
                                    hasUnknownTag = true;
                                }
                            }

                            if (hasUnknownTag || (hasShootTag && hasHoldTag)) {
                                telemetry.addLine("Mixed or unknown tags; decision is uncertain.");
                            } else {
                                frameCandidate = hasShootTag ? SHOOT_ALLOWED : HOLD;
                                frameTimestamp = result.getTimestamp();
                            }
                        }
                    }
                }

                if (frameCandidate.equals(UNCERTAIN)
                        || result == null
                        || result.getStaleness() > MAX_RESULT_AGE_MS) {
                    lastCandidate = UNCERTAIN;
                    stableFrames = 0;
                } else if (Double.compare(frameTimestamp, lastTimestamp) != 0) {
                    stableFrames = frameCandidate.equals(lastCandidate) ? stableFrames + 1 : 1;
                    lastCandidate = frameCandidate;
                    lastTimestamp = frameTimestamp;
                } else if (!frameCandidate.equals(lastCandidate)) {
                    lastCandidate = UNCERTAIN;
                    stableFrames = 0;
                }

                String decision = stableFrames >= REQUIRED_STABLE_FRAMES
                        && frameCandidate.equals(lastCandidate)
                        ? lastCandidate
                        : UNCERTAIN;
                telemetry.addData("Decision", decision);
                telemetry.addData("Stable frames", "%d / %d",
                        Math.min(stableFrames, REQUIRED_STABLE_FRAMES), REQUIRED_STABLE_FRAMES);
                telemetry.update();
                sleep(50);
            }
        } finally {
            limelight.stop();
        }
    }

    private static String getTagClassification(int tagId) {
        if (tagId >= 30 && tagId <= 33) {
            return "Red | Scoring";
        }
        if (tagId >= 34 && tagId <= 37) {
            return "Red | Audience";
        }
        if (tagId >= 38 && tagId <= 41) {
            return "Blue | Audience";
        }
        if (tagId >= 42 && tagId <= 45) {
            return "Blue | Scoring";
        }
        return "Unknown color/location";
    }

    private static boolean isShootTag(int tagId) {
        return (tagId >= 34 && tagId <= 37) || (tagId >= 42 && tagId <= 45);
    }

    private static boolean isKnownTag(int tagId) {
        return tagId >= 30 && tagId <= 45;
    }
}
