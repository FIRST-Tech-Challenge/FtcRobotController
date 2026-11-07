package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/**
 * Robot-centric mecanum drive with a Limelight 3A detecting colored balls.
 * Motor names and directions come from Constants.drivetrainConfig.
 *
 * Driving:
 *   left stick      forward / strafe
 *   right stick X   turn
 *   right bumper    slow mode (40%)
 *   left trigger    hold to auto-aim: the robot turns to face the biggest ball
 *                   while you keep control of forward / strafe
 *   right trigger   hold to follow: the robot turns toward the biggest ball and
 *                   drives up to it, stopping at FOLLOW_TARGET_AREA. The sticks
 *                   are ignored; let go to take back control. With no ball in
 *                   view the robot stops.
 *
 * Ball color (Limelight pipeline):
 *   X  blue     B  red     Y  yellow
 *
 * The Limelight switches pipelines by slot number, not by name, so the
 * PIPELINE_* constants below must match the slots "blue", "red" and "yellow"
 * are saved in on the camera. Telemetry shows the slot the camera reports.
 */
@TeleOp(name = "Ball Detect Test", group = "Test")
@Disabled
public class BallDetectTest extends LinearOpMode {

    private static final String LIMELIGHT_NAME = "limelight";

    private static final int PIPELINE_BLUE = 0 ;
    private static final int PIPELINE_RED = 1;
    private static final int PIPELINE_YELLOW = 2;

    private static final double SLOW_SCALE = 0.4;

    // Auto-aim: turn power per degree of target offset, and the most it may use
    private static final double AIM_KP = 0.02;
    private static final double AIM_MAX_TURN = 0.4;
    // Close enough to centered that we stop turning
    private static final double AIM_TOLERANCE_DEG = 1.0;
    // Follow: the ball's size in the image (% of frame) stands in for distance.
    // Stop when the ball fills this much of the image; raise it to stop closer.
    private static final double FOLLOW_TARGET_AREA = 5.0;
    // Within this much of the target area counts as arrived
    private static final double FOLLOW_AREA_TOLERANCE = 0.5;
    // Forward power per % of area still to go, and the most it may use
    private static final double FOLLOW_KP = 0.1;
    private static final double FOLLOW_MAX_FORWARD = 0.4;
    // Forward speed fades to zero as the ball gets this far off center, so the
    // robot turns to face the ball before driving at it
    private static final double FOLLOW_TURN_FIRST_DEG = 20.0;

    // Results older than this are ignored
    private static final long MAX_STALENESS_MS = 100;

    private enum BallColor {
        BLUE("Blue", PIPELINE_BLUE),
        RED("Red", PIPELINE_RED),
        YELLOW("Yellow", PIPELINE_YELLOW);

        final String label;
        final int pipeline;

        BallColor(String label, int pipeline) {
            this.label = label;
            this.pipeline = pipeline;
        }
    }

    @Override
    public void runOpMode() {
        MecanumConfig c = Constants.drivetrainConfig;
        DcMotorEx fL = initMotor(c.frontLeftName.get(), c.frontLeftDirection.get());
        DcMotorEx fR = initMotor(c.frontRightName.get(), c.frontRightDirection.get());
        DcMotorEx bL = initMotor(c.backLeftName.get(), c.backLeftDirection.get());
        DcMotorEx bR = initMotor(c.backRightName.get(), c.backRightDirection.get());

        Limelight3A limelight = null;
        String limelightError = null;
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        } catch (Exception e) {
            limelightError = e.getMessage();
        }

        BallColor color = BallColor.YELLOW;
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(color.pipeline);
            limelight.start();
        }

        telemetry.addLine("Ready. X = blue, B = red, Y = yellow, hold LT to aim, RT to follow.");
        if (limelight == null) telemetry.addData("Limelight", "NOT FOUND: %s", limelightError);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            BallColor requested = gamepad1.x ? BallColor.BLUE
                    : gamepad1.b ? BallColor.RED
                    : gamepad1.y ? BallColor.YELLOW
                    : color;
            if (requested != color && limelight != null) {
                limelight.pipelineSwitch(requested.pipeline);
            }
            color = requested;

            // Pick the biggest (usually closest) ball from a fresh result on the right pipeline
            LLResult result = limelight == null ? null : limelight.getLatestResult();
            boolean fresh = result != null && result.isValid()
                    && result.getStaleness() < MAX_STALENESS_MS
                    && result.getPipelineIndex() == color.pipeline;
            List<LLResultTypes.ColorResult> balls = fresh ? result.getColorResults() : null;
            LLResultTypes.ColorResult target = null;
            if (balls != null) {
                for (LLResultTypes.ColorResult ball : balls) {
                    if (target == null || ball.getTargetArea() > target.getTargetArea()) target = ball;
                }
            }

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            boolean following = gamepad1.right_trigger > 0.5;
            boolean aiming = !following && gamepad1.left_trigger > 0.5;
            String aimStatus = "off (hold LT to aim, RT to follow)";
            if (following) {
                forward = 0;
                strafe = 0;
                turn = 0;
                if (target == null) {
                    aimStatus = "FOLLOW: no ball in view, stopped";
                } else {
                    double tx = target.getTargetXDegrees();
                    double areaError = FOLLOW_TARGET_AREA - target.getTargetArea();
                    if (Math.abs(tx) >= AIM_TOLERANCE_DEG) {
                        turn = Range.clip(AIM_KP * tx, -AIM_MAX_TURN, AIM_MAX_TURN);
                    }
                    if (Math.abs(areaError) >= FOLLOW_AREA_TOLERANCE) {
                        // Too small means too far: drive forward. Too big means too close: back up.
                        double facing = Math.max(0, 1 - Math.abs(tx) / FOLLOW_TURN_FIRST_DEG);
                        forward = facing * Range.clip(FOLLOW_KP * areaError,
                                -FOLLOW_MAX_FORWARD, FOLLOW_MAX_FORWARD);
                    }
                    aimStatus = forward == 0 && turn == 0 ? "FOLLOW: arrived"
                            : String.format("FOLLOW: driving (area %.2f%% of %.2f%%)",
                                    target.getTargetArea(), FOLLOW_TARGET_AREA);
                }
            } else if (aiming) {
                if (target == null) {
                    aimStatus = "no ball in view";
                } else {
                    // Positive tx means the ball is right of center, and positive turn turns right
                    double tx = target.getTargetXDegrees();
                    turn = Math.abs(tx) < AIM_TOLERANCE_DEG ? 0
                            : Range.clip(AIM_KP * tx, -AIM_MAX_TURN, AIM_MAX_TURN);
                    aimStatus = Math.abs(tx) < AIM_TOLERANCE_DEG ? "LOCKED" : "turning";
                }
            }

            double flPower = forward + strafe + turn;
            double frPower = forward - strafe - turn;
            double blPower = forward - strafe + turn;
            double brPower = forward + strafe - turn;

            // Keep ratios between wheels when any power exceeds 1
            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower))));
            double scale = (gamepad1.right_bumper ? SLOW_SCALE : 1.0) / max;
            fL.setPower(flPower * scale);
            fR.setPower(frPower * scale);
            bL.setPower(blPower * scale);
            bR.setPower(brPower * scale);

            telemetry.addData("Looking for", "%s (pipeline %d)", color.label, color.pipeline);
            telemetry.addData("Auto-aim", aimStatus);
            addLimelightTelemetry(limelight, limelightError, result, fresh, balls, target);
            telemetry.update();
        }

        if (limelight != null) limelight.stop();
    }

    private void addLimelightTelemetry(Limelight3A limelight, String error, LLResult result, boolean fresh,
                                       List<LLResultTypes.ColorResult> balls,
                                       LLResultTypes.ColorResult target) {
        telemetry.addLine();
        if (limelight == null) {
            telemetry.addData("Limelight", "NOT FOUND: %s", error);
            return;
        }
        if (!limelight.isConnected()) {
            telemetry.addData("Limelight", "not connected");
            return;
        }

        LLStatus status = limelight.getStatus();
        telemetry.addData("Camera pipeline", "%d (%s)", status.getPipelineIndex(), status.getPipelineType());
        telemetry.addData("FPS / CPU / temp", "%.0f / %.0f%% / %.1fC",
                status.getFps(), status.getCpu(), status.getTemp());
        if (result != null) telemetry.addData("Result age (ms)", result.getStaleness());

        if (!fresh || balls == null || balls.isEmpty()) {
            telemetry.addData("Balls", "none");
            return;
        }
        telemetry.addData("Balls", balls.size());
        telemetry.addData("Target", "tx %.1f  ty %.1f  area %.2f%%",
                target.getTargetXDegrees(), target.getTargetYDegrees(), target.getTargetArea());
        for (int i = 0; i < balls.size(); i++) {
            LLResultTypes.ColorResult ball = balls.get(i);
            telemetry.addData("  Ball " + (i + 1), "tx %.1f  ty %.1f  area %.2f%%",
                    ball.getTargetXDegrees(), ball.getTargetYDegrees(), ball.getTargetArea());
        }
    }

    private DcMotorEx initMotor(String name, DcMotor.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
}
