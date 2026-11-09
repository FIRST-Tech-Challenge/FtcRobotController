package org.firstinspires.ftc.teamcode.attic;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Limelight tester with an "Aiming Mode".
 *
 * Normal: robot-relative mecanum drive.
 * Hold Y: Aiming Mode. The sticks are ignored and the robot turns in place to face
 *         the hive tags. Let go of Y to get normal driving back.
 *
 * During INIT press X for BLUE alliance or B for RED (default BLUE).
 *
 * Aiming uses tx (the tag's left/right angle in the camera image) to turn, and the
 * tag's "Target Pose in Robot Space" for distance. For the distance to be right,
 * the camera's position on the robot MUST be entered in the Limelight web UI.
 */
@Disabled
@TeleOp(name = "Limelight Test Orange", group = "Test")
public class LimelightTestOrange extends LinearOpMode {

    // ---------------- Tuning / configuration ----------------
    static final int PIPELINE = 0;

    // Hive tag IDs for each alliance. TODO: fill in from the game manual.
    // While a list is empty, ANY tag the Limelight sees is used.
    static final int[] BLUE_HIVE_TAG_IDS = {
            38, 39, 40, 41, // Audience side
            42, 43, 44, 45  // Scoring side
    };
    static final int[] RED_HIVE_TAG_IDS = {
            30, 31, 32, 33, // Scoring side
            34, 35, 36, 37  // Audience side    
    };

    static final double TURN_KP = 0.02;          // turn power per degree of tx
    static final double TURN_MIN_POWER = 0.06;   // enough to overcome friction
    static final double TURN_MAX_POWER = 0.5;
    static final double AIM_TOLERANCE_DEG = 1.5; // "on target" if |tx| is below this
    static final double MAX_STALENESS_MS = 100;  // ignore results older than this
    static final double METERS_TO_INCHES = 39.3701;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor intake;
    Limelight3A limelight;

    boolean blueAlliance = true;
    boolean wasOnTarget = false;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {

        // Find the four drive motors by the names in the robot configuration.
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");
        intake = hardwareMap.dcMotor.get("intake_motor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pick the alliance during INIT.
        while (opModeInInit()) {
            if (gamepad1.x) blueAlliance = true;
            if (gamepad1.b) blueAlliance = false;
            telemetry.addLine("Press START + A on your gamepad to connect it as gamepad1.");
            telemetry.addLine("Alliance: X = BLUE, B = RED");
            telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
            telemetry.addLine("Hold Y during the match for Aiming Mode.");
            telemetry.update();
        }

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(PIPELINE);

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            HiveSighting hive = findHive(result);
            boolean aiming = gamepad1.y; // hold Y for aiming mode

            if (aiming) {
                aim(hive);
            } else {
                wasOnTarget = false;
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = -1.0f * gamepad1.right_stick_x;
                drive(forward, strafe, turn);
                if (gamepad1.right_bumper) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
                telemetry.addData("Drive", String.format(
                        "fwd: %.1f  strafe: %.1f  turn: %.1f", forward, strafe, turn));
            }

            telemetry.addData("Info", "Hold Y to aim");
            telemetry.addData("Mode", aiming ? "AIMING (sticks disabled)" : "DRIVING");
            telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
            if (hive != null) {
                telemetry.addData("LL Pipeline", PIPELINE);
                telemetry.addData("Hive tags seen", hive.tagCount);
                telemetry.addData("tx (deg)", String.format("%.2f", hive.txDeg));
                telemetry.addData("Distance (in)", String.format("%.1f", hive.distanceMeters * METERS_TO_INCHES));
                // Raw robot-space numbers. Use these to check which axis is forward / left / up.
                telemetry.addData("Raw robot-space x,y,z (m)", String.format(
                        "%.2f, %.2f, %.2f", hive.x, hive.y, hive.z));
            } else {
                telemetry.addData("Hive", "No hive tags seen");
            }
            telemetry.update();
        }

        drive(0, 0, 0);
        limelight.stop();
    }

    /** Aiming Mode: turn in place toward the hive. No manual driving while aiming. */
    private void aim(HiveSighting hive) {
        if (hive == null) {
            // Can't see the hive: stay still rather than spin around looking for it.
            drive(0, 0, 0);
            wasOnTarget = false;
            telemetry.addData("Aim", "NO TARGET - point the robot at the hive");
            return;
        }

        double error = hive.txDeg; // + means the target is to the right
        boolean onTarget = Math.abs(error) < AIM_TOLERANCE_DEG;

        double turn = 0;
        if (!onTarget) {
            // Turning right (clockwise) is NEGATIVE turn in drive().
            double power = TURN_KP * Math.abs(error);
            power = Range.clip(power, TURN_MIN_POWER, TURN_MAX_POWER);
            //turn = -Math.signum(error) * power; // TODO: invert this and test
            turn = Math.signum(error) * power; // here's the test

        }
        drive(0, 0, turn);

        // Buzz the driver's controller once when we first line up.
        if (onTarget && !wasOnTarget) {
            gamepad1.rumble(200); // does this work?
        }
        wasOnTarget = onTarget;

        telemetry.addData("Aim", onTarget ? "ON TARGET" : String.format("turning %.2f", turn));
    }

    /** What we know about the hive from one Limelight frame. */
    static class HiveSighting {
        double txDeg;          // average horizontal angle to the hive tags
        double distanceMeters; // average straight-line distance, camera mount corrected
        double x, y, z;        // average target position in robot space (meters)
        int tagCount;
    }

    /**
     * Look through every tag in the frame, keep the ones on our alliance's hive,
     * and average them so we aim at the middle of the group of tags we can see.
     * Returns null if no usable tags.
     */
    private HiveSighting findHive(LLResult result) {
        if (result == null || !result.isValid() || result.getStaleness() > MAX_STALENESS_MS) {
            return null;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return null;

        HiveSighting hive = new HiveSighting();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (!isOurHiveTag(tag.getFiducialId())) continue;

            Pose3D pose = tag.getTargetPoseRobotSpace();
            Position p = pose.getPosition(); // Limelight reports meters
            hive.txDeg += tag.getTargetXDegrees();
            hive.x += p.x;
            hive.y += p.y;
            hive.z += p.z;
            // Straight-line distance doesn't depend on which axis is which.
            hive.distanceMeters += Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            hive.tagCount++;
        }
        if (hive.tagCount == 0) return null;

        hive.txDeg /= hive.tagCount;
        hive.x /= hive.tagCount;
        hive.y /= hive.tagCount;
        hive.z /= hive.tagCount;
        hive.distanceMeters /= hive.tagCount;
        return hive;
    }

    private boolean isOurHiveTag(int id) {
        int[] ids = blueAlliance ? BLUE_HIVE_TAG_IDS : RED_HIVE_TAG_IDS;
        if (ids.length == 0) return true; // IDs not filled in yet: accept any tag
        for (int good : ids) {
            if (good == id) return true;
        }
        return false;
    }

    // standard mecanum drive
    private void drive(double forward, double right, double turn) {
        double fl = forward + right - turn;
        double bl = forward - right - turn;
        double fr = forward - right + turn;
        double br = forward + right + turn;

        // Scale all four down together if any exceeds 1, so the direction is kept, just slower.
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }
}
