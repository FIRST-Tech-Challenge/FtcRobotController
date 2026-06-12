package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Tank-drive TeleOp for a rear-wheel-drive base with two motors per side.
 *
 * Physical wiring (all on Control Hub):
 *   Port 0 — rightBack  (RB)
 *   Port 1 — leftBack   (LB)
 *   Port 2 — rightFront (RF)
 *   Port 3 — leftFront  (LF)
 *
 * LF + LB are geared → belt → left rear wheel.
 * RF + RB are geared → belt → right rear wheel.
 * Front wheels are passive (not driven).
 *
 * Controls (Gamepad 1):
 *   Left  stick Y  — throttle (forward / reverse)
 *   Left  stick X  — steering (left / right)
 *   Left  bumper   — slow mode (50% power cap)
 *   Right bumper   — drift mode: motors coast (FLOAT) + squared inputs on
 *                    both axes for sharp throttle/steer response; flick X
 *                    mid-throttle to induce oversteer, counter-steer to catch
 */
@TeleOp(name = "Drivebase", group = "TeleOp")
public class Drivebase extends LinearOpMode {

    // Motor declarations — names must match Driver Hub configuration
    private DcMotor leftFront;   // port 3
    private DcMotor rightFront;  // port 2
    private DcMotor leftBack;    // port 1
    private DcMotor rightBack;   // port 0

    private static final double SLOW_MODE_FACTOR = 0.5;

    private boolean prevDriftMode = false;

    @Override
    public void runOpMode() {

        // ── Hardware map ──────────────────────────────────────────────────────
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBackMotor");

        // ── Motor directions ──────────────────────────────────────────────────
        // Left side: reverse so positive power drives the robot forward.
        // If your left motors spin the wrong way, swap REVERSE ↔ FORWARD here.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Right side: forward so positive power drives the robot forward.
        // If your right motors spin the wrong way, swap FORWARD ↔ REVERSE here.
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Note: if the two motors on one side are mounted facing opposite
        // directions in the gearbox, reverse ONE of them relative to the other
        // (e.g. leftFront FORWARD, leftBack REVERSE) so they pull together.

        // ── Zero-power behavior ───────────────────────────────────────────────
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Run mode ──────────────────────────────────────────────────────────
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();

        // ── Main loop ─────────────────────────────────────────────────────────
        while (opModeIsActive()) {

            boolean driftMode = gamepad1.right_bumper;

            // Switch zero-power behavior only when drift mode toggles to avoid
            // unnecessary I2C traffic every loop iteration.
            if (driftMode != prevDriftMode) {
                setZeroPowerBehavior(driftMode
                        ? DcMotor.ZeroPowerBehavior.FLOAT
                        : DcMotor.ZeroPowerBehavior.BRAKE);
                prevDriftMode = driftMode;
            }

            // Negate Y because gamepad Y-axis is inverted (up = -1).
            double drive =  -gamepad1.left_stick_y;
            double turn  =   gamepad1.left_stick_x;

            // Drift mode: square each axis independently (preserve sign) before
            // mixing — small inputs stay subtle, full deflection snaps to 100%,
            // matching the hair-trigger throttle/steer feel of real drift driving.
            if (driftMode) {
                drive = Math.signum(drive) * drive * drive;
                turn  = Math.signum(turn)  * turn  * turn;
            }

            // Arcade mix: one side adds the turn, the other subtracts it.
            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Normalize so the mix never exceeds motor limits.
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower  /= maxPower;
                rightPower /= maxPower;
            }

            // Slow mode: hold left bumper for fine control.
            if (gamepad1.left_bumper) {
                leftPower  *= SLOW_MODE_FACTOR;
                rightPower *= SLOW_MODE_FACTOR;
            }

            setLeftPower(leftPower);
            setRightPower(rightPower);

            telemetry.addData("Drive",      "%.2f", drive);
            telemetry.addData("Turn",       "%.2f", turn);
            telemetry.addData("Left power", "%.2f", leftPower);
            telemetry.addData("Right power","%.2f", rightPower);
            telemetry.addData("Slow mode",  gamepad1.left_bumper);
            telemetry.addData("Drift mode", driftMode);
            telemetry.update();
        }

        // Stop all motors when OpMode ends.
        setLeftPower(0);
        setRightPower(0);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private void setLeftPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
    }

    private void setRightPower(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        leftFront.setZeroPowerBehavior(zpb);
        leftBack.setZeroPowerBehavior(zpb);
        rightFront.setZeroPowerBehavior(zpb);
        rightBack.setZeroPowerBehavior(zpb);
    }
}
