package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;

/**
 * Disabled, uncalibrated prototype: rotate the chassis toward one selected AprilTag.
 * Hold A to aim in place; release A or move the right stick to resume manual driving.
 * Uses OctoQuad heading between camera frames, not field position or MegaTag localization.
 * No shooter control, range solution, camera latency compensation, or hardware acceptance.
 *
 * Before enabling: select the pipeline/tag, calibrate the camera-to-shooter offset,
 * verify BOTH rotation signs at low power, then tune gains and test loss of feedback.
 * The default tag ID intentionally accepts no target. A tag's center is not necessarily
 * the scoring opening: AIM_TX_DEG must be calibrated for the actual shooting position.
 */
@Disabled
@TeleOp(name = "PROTOTYPE - Turretless Aim", group = "Testing")
public final class TurretlessAimPrototype extends LinearOpMode {
    private static final int PIPELINE = 0;
    private static final int TARGET_TAG_ID = -1;
    private static final double AIM_TX_DEG = 0.0;
    // Assumptions to VERIFY: tx is positive right; heading is positive counterclockwise;
    // Drive.drive's positive turn rotates clockwise on the assembled robot.
    private static final double CAMERA_TO_HEADING_SIGN = -1.0;
    private static final double HEADING_TO_TURN_SIGN = -1.0;
    private static final double TURN_KP = 0.015; // power per degree; not tuned
    private static final double MAX_TURN = 0.20;
    private static final double TOLERANCE_DEG = 1.5;
    private static final double SETTLE_SECONDS = 0.20;
    private static final double MAX_FRAME_AGE_MS = 100.0;
    private static final double MANUAL_SCALE = 0.25;
    private static final double MANUAL_OVERRIDE = 0.12;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        // These subsystems are polled explicitly in this standalone LinearOpMode.
        CommandScheduler.getInstance().unregisterSubsystem(drive);
        Limelight3A camera = null;
        try {
            Localizer localizer = new Localizer(hardwareMap);
            CommandScheduler.getInstance().unregisterSubsystem(localizer);
            camera = hardwareMap.get(Limelight3A.class, "limelight");
            camera.pipelineSwitch(PIPELINE);
            camera.setPollRateHz(50);
            camera.start();

            while (opModeInInit()) {
                localizer.periodic();
                telemetry.addData("OctoQuad", localizer.getStatus());
                telemetry.addData("Selected tag", TARGET_TAG_ID);
                telemetry.addLine("UNCALIBRATED prototype. Keep still for OctoQuad calibration.");
                telemetry.addLine("Hold A: aim in place. Release A / right stick: manual.");
                telemetry.update();
                sleep(20);
            }
            if (isStopRequested()) {
                return;
            }

            double lastFrameStamp = Double.NaN;
            double lastFrameChange = Double.NEGATIVE_INFINITY;
            double targetHeading = Double.NaN;
            double settledSince = Double.NaN;

            while (opModeIsActive()) {
                double now = getRuntime();
                localizer.periodic();
                localizer.captureStartHeading();
                double heading = Math.toDegrees(localizer.getHeadingRadians());
                LLResult result = camera.getLatestResult();
                double tx = selectedTargetX(result);
                double stamp = result == null ? Double.NaN : result.getTimestamp();
                boolean newFrame = Double.isFinite(stamp) && stamp != lastFrameStamp;
                if (newFrame) {
                    lastFrameStamp = stamp;
                    lastFrameChange = now;
                }
                // Check both receive age and frame progression; repeated polls are not new frames.
                boolean fresh = result != null && Double.isFinite(stamp)
                        && result.getStaleness() >= 0 && result.getStaleness() <= MAX_FRAME_AGE_MS
                        && (now - lastFrameChange) * 1000.0 <= MAX_FRAME_AGE_MS;
                boolean manual = !gamepad1.a || Math.abs(gamepad1.right_stick_x) > MANUAL_OVERRIDE;
                double turn = 0.0;
                double error = Double.NaN;
                String state;

                if (manual) {
                    targetHeading = Double.NaN;
                    settledSince = Double.NaN;
                    // Match the manual turn convention in the current GamepadConfig.
                    turn = gamepad1.right_stick_x * MANUAL_SCALE;
                    drive.drive(-gamepad1.left_stick_y * MANUAL_SCALE,
                            gamepad1.left_stick_x * MANUAL_SCALE, turn);
                    state = "MANUAL";
                } else if (!Double.isFinite(heading) || !fresh || !Double.isFinite(tx)) {
                    drive.stop();
                    targetHeading = Double.NaN;
                    settledSince = Double.NaN;
                    state = "WAITING FOR VALID HEADING / FRESH SELECTED TAG";
                } else {
                    // One heading target per camera frame. Between frames the OctoQuad closes
                    // the rotation loop; do not add the same old tx to each new heading sample.
                    if (newFrame || !Double.isFinite(targetHeading)) {
                        targetHeading = wrapDegrees(heading
                                + CAMERA_TO_HEADING_SIGN * (tx - AIM_TX_DEG));
                    }
                    error = wrapDegrees(targetHeading - heading);
                    boolean inTolerance = Math.abs(error) <= TOLERANCE_DEG
                            && Math.abs(tx - AIM_TX_DEG) <= TOLERANCE_DEG;
                    if (inTolerance) {
                        if (!Double.isFinite(settledSince)) {
                            settledSince = now;
                        }
                        state = now - settledSince >= SETTLE_SECONDS ? "ALIGNED" : "SETTLING";
                    } else {
                        settledSince = Double.NaN;
                        turn = Math.max(-MAX_TURN, Math.min(MAX_TURN,
                                HEADING_TO_TURN_SIGN * TURN_KP * error));
                        state = "AIMING";
                    }
                    // No translation while aiming: this prototype only addresses rotation.
                    drive.drive(0.0, 0.0, turn);
                }

                telemetry.addData("Prototype state", state);
                telemetry.addData("OctoQuad", localizer.getStatus());
                telemetry.addData("Heading (deg)", heading);
                telemetry.addData("Selected tag tx (deg)", tx);
                telemetry.addData("Fresh frame", fresh);
                telemetry.addData("Target heading (deg)", targetHeading);
                telemetry.addData("Heading error (deg)", error);
                telemetry.addData("Turn command", turn);
                telemetry.addLine("ALIGNED is angular alignment only, not permission to fire.");
                telemetry.update();
                sleep(10);
            }
        } finally {
            try {
                drive.stop();
            } finally {
                if (camera != null) {
                    camera.stop();
                }
            }
        }
    }

    private static double selectedTargetX(LLResult result) {
        if (TARGET_TAG_ID < 0 || result == null || !result.isValid()
                || result.getPipelineIndex() != PIPELINE) {
            return Double.NaN;
        }
        for (LLResultTypes.FiducialResult target : result.getFiducialResults()) {
            if (target.getFiducialId() == TARGET_TAG_ID) {
                return target.getTargetXDegrees();
            }
        }
        return Double.NaN;
    }

    private static double wrapDegrees(double degrees) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)),
                Math.cos(Math.toRadians(degrees))));
    }
}
