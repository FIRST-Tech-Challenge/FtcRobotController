package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Layer 2 Standalone Test OpMode: Tests ONLY the MecanumDrivetrain subsystem in isolation
 * (does not require the Pinpoint or any other mechanism to be wired).
 *
 * Controls (Gamepad 1):
 * - Left Stick: Translate (forward/backward & strafe)
 * - Right Stick X: Rotate
 * - Right Bumper + X/Y/A/B: Spin a single wheel at 25% power (X=FL, Y=FR, A=BL, B=BR) to verify wiring
 * - X/Y/A/B (no RB): Repeatable fixed-power drive through the mecanum mix
 *   (Y=forward, A=backward, X=strafe left, B=strafe right) for comparing wheel speeds under load
 */
@TeleOp(name = "Test: Drivetrain Only", group = "Test")
public class DrivetrainTestOpMode extends LinearOpMode {

    private static final double SINGLE_WHEEL_TEST_POWER = 0.25;
    private static final double FIXED_DRIVE_TEST_POWER = 0.30;

    @Override
    public void runOpMode() {
        RobotLoop loop = new RobotLoop(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);

        telemetry.addData("Status", "Drivetrain Test Initialized");
        drivetrain.addConfigurationTelemetry(telemetry);
        telemetry.addLine("Normal Drive: Left Stick = Translate | Right Stick X = Rotate");
        telemetry.addLine("Single Wheel Test: Hold RB + X (FL), Y (FR), A (BL), or B (BR)");
        telemetry.addLine("Fixed Drive Test (30%): Hold Y fwd, A back, X left, B right");
        loop.addTelemetry(telemetry); telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                loop.begin();
                if (gamepad1.right_bumper) {
                    double fl = gamepad1.x ? SINGLE_WHEEL_TEST_POWER : 0.0;
                    double fr = gamepad1.y ? SINGLE_WHEEL_TEST_POWER : 0.0;
                    double bl = gamepad1.a ? SINGLE_WHEEL_TEST_POWER : 0.0;
                    double br = gamepad1.b ? SINGLE_WHEEL_TEST_POWER : 0.0;
                    drivetrain.setWheelPowers(fl, fr, bl, br);
                    telemetry.addLine("Mode: Single Wheel Test (RB held)");
                } else if (gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b) {
                    double axial = gamepad1.y ? FIXED_DRIVE_TEST_POWER
                            : gamepad1.a ? -FIXED_DRIVE_TEST_POWER : 0.0;
                    double lateral = gamepad1.b ? FIXED_DRIVE_TEST_POWER
                            : gamepad1.x ? -FIXED_DRIVE_TEST_POWER : 0.0;
                    drivetrain.driveRobotCentric(axial, lateral, 0.0);
                    telemetry.addData("Mode", "Fixed Drive Test (axial %.2f | lat %.2f)", axial, lateral);
                } else {
                    double axial = -gamepad1.left_stick_y;
                    double lateral = gamepad1.left_stick_x;
                    double yaw = gamepad1.right_stick_x;
                    drivetrain.driveRobotCentric(axial, lateral, yaw);
                    telemetry.addData("Mode", "Mecanum Drive (axial %.2f | lat %.2f | yaw %.2f)",
                            axial, lateral, yaw);
                }

                drivetrain.addTelemetry(telemetry);
                loop.addTelemetry(telemetry); telemetry.update();
            }
        } finally {
            drivetrain.stop();
        }
    }
}
