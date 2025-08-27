package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasHardware;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasMovementAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.Point;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.RobotState;


@TeleOp(name="Archytas: PID Tuning", group="Archytas")
public class ArchytasPIDTuning extends LinearOpMode {

    private ArchytasHardware hardware = new ArchytasHardware();
    private ArchytasMovementAlgorithm movement = new ArchytasMovementAlgorithm();

    // PID Gains (adjust these during testing)
    private double pathKp = 0.8, pathKi = 0.0, pathKd = 0.1;
    private double headingKp = 1.2, headingKi = 0.0, headingKd = 0.05;
    private double xKp = 0.6, xKi = 0.0, xKd = 0.08;
    private double yKp = 0.6, yKi = 0.0, yKd = 0.08;

    private String currentController = "heading";
    private Point targetPosition = new Point(24, 0);
    private double targetHeading = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        if (!hardware.init(hardwareMap)) {
            telemetry.addData("ERROR", "Hardware initialization failed!");
            telemetry.update();
            while (opModeIsActive()) sleep(100);
            return;
        }

        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Target", "24 inches forward");
        telemetry.addData("Controller", currentController);
        telemetry.update();

        waitForStart();

        hardware.resetEncoders();
        hardware.resetIMUHeading();
        movement.setPosition(new Point(0, 0));
        movement.setHeading(0);
        movement.setTarget(targetPosition);

        while (opModeIsActive()) {
            double deltaTime = hardware.getDeltaTime();

            // Update odometry
            movement.updateOdometry(
                    hardware.getLeftEncoderPosition(),
                    hardware.getRightEncoderPosition(),
                    hardware.getPerpEncoderPosition(),
                    hardware.getIMUHeading(),
                    deltaTime
            );

            RobotState currentState = movement.getCurrentState();

            // PID Tuning controls
            handlePIDTuning();

            // Update PID gains
            movement.setPIDGains("path", pathKp, pathKi, pathKd);
            movement.setPIDGains("heading", headingKp, headingKi, headingKd);
            movement.setPIDGains("x", xKp, xKi, xKd);
            movement.setPIDGains("y", yKp, yKi, yKd);

            // Run autonomous movement
            double[] powers = movement.autonomousPointToPoint(currentState, targetHeading, deltaTime);

            // Override with manual control if needed
            if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                double drive = -gamepad1.left_stick_y * 0.5;
                double strafe = gamepad1.left_stick_x * 0.5;
                double turn = gamepad1.right_stick_x * 0.5;

                powers[0] = drive + strafe + turn; // Front left
                powers[1] = drive - strafe - turn; // Front right
                powers[2] = drive - strafe + turn; // Back left
                powers[3] = drive + strafe - turn; // Back right

                // Normalize
                double max = Math.max(1.0, Math.max(
                        Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                        Math.max(Math.abs(powers[2]), Math.abs(powers[3]))
                ));
                for (int i = 0; i < 4; i++) powers[i] /= max;
            }

            hardware.setMotorPowers(powers);

            // Reset position if requested
            if (gamepad1.start) {
                hardware.resetEncoders();
                hardware.resetIMUHeading();
                movement.setPosition(new Point(0, 0));
                movement.setHeading(0);
                movement.setTarget(targetPosition);
            }

            // Telemetry
            Point position = currentState.getPosition();
            double distance = position.distanceTo(targetPosition);
            double headingError = Math.toDegrees(targetHeading - currentState.heading);

            telemetry.addData("=== PID TUNING ===", "");
            telemetry.addData("Controller", currentController.toUpperCase());

            if (currentController.equals("heading")) {
                telemetry.addData("Kp", "%.3f", headingKp);
                telemetry.addData("Ki", "%.3f", headingKi);
                telemetry.addData("Kd", "%.3f", headingKd);
            } else if (currentController.equals("x")) {
                telemetry.addData("Kp", "%.3f", xKp);
                telemetry.addData("Ki", "%.3f", xKi);
                telemetry.addData("Kd", "%.3f", xKd);
            } else if (currentController.equals("y")) {
                telemetry.addData("Kp", "%.3f", yKp);
                telemetry.addData("Ki", "%.3f", yKi);
                telemetry.addData("Kd", "%.3f", yKd);
            } else { // path
                telemetry.addData("Kp", "%.3f", pathKp);
                telemetry.addData("Ki", "%.3f", pathKi);
                telemetry.addData("Kd", "%.3f", pathKd);
            }

            telemetry.addData("", "");
            telemetry.addData("=== STATUS ===", "");
            telemetry.addData("Position", "X: %.1f, Y: %.1f", position.x, position.y);
            telemetry.addData("Target", "X: %.1f, Y: %.1f", targetPosition.x, targetPosition.y);
            telemetry.addData("Distance Error", "%.1f inches", distance);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentState.heading));
            telemetry.addData("Heading Error", "%.1f degrees", headingError);
            telemetry.addData("Complete", movement.isPathComplete() ? "YES" : "NO");
            telemetry.addData("", "");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("DPAD", "Adjust gains (+/- 0.1)");
            telemetry.addData("A/B/X/Y", "Select controller");
            telemetry.addData("Bumpers", "Fine adjust (+/- 0.01)");
            telemetry.addData("Start", "Reset position");
            telemetry.addData("Sticks", "Manual override");

            telemetry.update();
            sleep(50);
        }

        hardware.stopMotors();
    }

    private void handlePIDTuning() {
        // Controller selection
        if (gamepad1.a && !gamepad1.start) { // Avoid conflict with reset
            currentController = "heading";
            sleep(200);
        } else if (gamepad1.b) {
            currentController = "x";
            sleep(200);
        } else if (gamepad1.x) {
            currentController = "y";
            sleep(200);
        } else if (gamepad1.y) {
            currentController = "path";
            sleep(200);
        }

        // Gain adjustments
        double coarseStep = 0.1;
        double fineStep = 0.01;
        double step = gamepad1.left_bumper || gamepad1.right_bumper ? fineStep : coarseStep;

        if (currentController.equals("heading")) {
            if (gamepad1.dpad_up) { headingKp += step; sleep(100); }
            if (gamepad1.dpad_down) { headingKp = Math.max(0, headingKp - step); sleep(100); }
            if (gamepad1.dpad_right) { headingKi += step; sleep(100); }
            if (gamepad1.dpad_left) { headingKi = Math.max(0, headingKi - step); sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_up) { headingKd += fineStep; sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_down) { headingKd = Math.max(0, headingKd - fineStep); sleep(100); }
        } else if (currentController.equals("x")) {
            if (gamepad1.dpad_up) { xKp += step; sleep(100); }
            if (gamepad1.dpad_down) { xKp = Math.max(0, xKp - step); sleep(100); }
            if (gamepad1.dpad_right) { xKi += step; sleep(100); }
            if (gamepad1.dpad_left) { xKi = Math.max(0, xKi - step); sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_up) { xKd += fineStep; sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_down) { xKd = Math.max(0, xKd - fineStep); sleep(100); }
        } else if (currentController.equals("y")) {
            if (gamepad1.dpad_up) { yKp += step; sleep(100); }
            if (gamepad1.dpad_down) { yKp = Math.max(0, yKp - step); sleep(100); }
            if (gamepad1.dpad_right) { yKi += step; sleep(100); }
            if (gamepad1.dpad_left) { yKi = Math.max(0, yKi - step); sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_up) { yKd += fineStep; sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_down) { yKd = Math.max(0, yKd - fineStep); sleep(100); }
        } else { // path
            if (gamepad1.dpad_up) { pathKp += step; sleep(100); }
            if (gamepad1.dpad_down) { pathKp = Math.max(0, pathKp - step); sleep(100); }
            if (gamepad1.dpad_right) { pathKi += step; sleep(100); }
            if (gamepad1.dpad_left) { pathKi = Math.max(0, pathKi - step); sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_up) { pathKd += fineStep; sleep(100); }
            if (gamepad1.right_bumper && gamepad1.dpad_down) { pathKd = Math.max(0, pathKd - fineStep); sleep(100); }
        }
    }
}
