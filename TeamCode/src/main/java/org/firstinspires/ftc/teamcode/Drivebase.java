package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Tank-drive TeleOp for a 4-wheel independently-driven base.
 *
 * Physical wiring:
 *   Port 0 (CH) — rightBack  (RB)
 *   Port 1 (CH) — leftBack   (LB)
 *   Port 2 (CH) — rightFront (RF)
 *   Port 3 (CH) — leftFront  (LF)
 *
 * All four wheels are independently driven.
 *
 * Controls (Gamepad 1):
 *   Left  stick Y  — throttle (forward / reverse)
 *   Right stick X  — steering (left / right)
 *   Left  bumper   — slow mode (50% power cap)
 *   Right bumper   — drift mode: motors coast (FLOAT) + squared inputs on
 *                    both axes for sharp throttle/steer response
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
        // Left side: forward so positive power drives the robot forward.
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Right side: forward so positive power drives the robot forward.
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

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

            // ── Drive Logic ──────────────────────────────────────────────────
            boolean driftMode = gamepad1.right_bumper;

            if (driftMode != prevDriftMode) {
                setZeroPowerBehavior(driftMode
                        ? DcMotor.ZeroPowerBehavior.FLOAT
                        : DcMotor.ZeroPowerBehavior.BRAKE);
                prevDriftMode = driftMode;
            }

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            if (driftMode) {
                drive = Math.signum(drive) * drive * drive;
                turn  = Math.signum(turn)  * turn  * turn;
            }

            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower  /= maxPower;
                rightPower /= maxPower;
            }

            if (gamepad1.left_bumper) {
                leftPower  *= SLOW_MODE_FACTOR;
                rightPower *= SLOW_MODE_FACTOR;
            }

            setLeftPower(leftPower);
            setRightPower(rightPower);

            // ── Telemetry ────────────────────────────────────────────────────
            telemetry.addData("Drive",      "%.2f", drive);
            telemetry.addData("Turn",       "%.2f", turn);
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
