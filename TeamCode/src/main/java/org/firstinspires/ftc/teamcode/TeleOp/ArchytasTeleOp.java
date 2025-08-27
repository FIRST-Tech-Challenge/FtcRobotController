package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasHardware;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasMovementAlgorithm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.Point;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.RobotState;




@TeleOp(name="Archytas: Enhanced TeleOp", group="Archytas")
    public class ArchytasTeleOp extends LinearOpMode {

        private ArchytasHardware hardware = new ArchytasHardware();
        private ArchytasMovementAlgorithm movement = new ArchytasMovementAlgorithm();

        private boolean fieldCentric = true;
        private double speedMultiplier = 1.0;

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initializing...");
            telemetry.update();

            // Initialize hardware
            if (!hardware.init(hardwareMap)) {
                telemetry.addData("ERROR", "Hardware initialization failed!");
                telemetry.update();
                while (opModeIsActive()) sleep(100);
                return;
            }

            telemetry.addData("Status", "Ready to start");
            telemetry.addData("Field Centric", fieldCentric ? "ON" : "OFF");
            telemetry.addData("Speed", "%.0f%%", speedMultiplier * 100);
            telemetry.update();

            waitForStart();
            hardware.resetIMUHeading(); // Reset heading at start

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

                // Get current robot state
                RobotState currentState = movement.getCurrentState();

                // Control inputs
                double drive = -gamepad1.left_stick_y * speedMultiplier;
                double strafe = gamepad1.left_stick_x * speedMultiplier;
                double turn = gamepad1.right_stick_x * speedMultiplier;

                // Toggle field centric
                if (gamepad1.back) {
                    fieldCentric = !fieldCentric;
                    sleep(200); // Debounce
                }

                // Speed control
                if (gamepad1.right_bumper) {
                    speedMultiplier = 0.5; // Slow mode
                } else if (gamepad1.left_bumper) {
                    speedMultiplier = 1.0; // Fast mode
                }

                // Reset heading
                if (gamepad1.start) {
                    hardware.resetIMUHeading();
                    movement.setHeading(0);
                }

                // Calculate motor powers using enhanced teleop
                double[] powers = movement.teleopUpdate(currentState, drive, strafe, turn, fieldCentric, deltaTime);
                hardware.setMotorPowers(powers);

                // Telemetry
                Point position = currentState.getPosition();
                telemetry.addData("Status", "Running");
                telemetry.addData("Field Centric", fieldCentric ? "ON (Back to toggle)" : "OFF (Back to toggle)");
                telemetry.addData("Speed Mode", speedMultiplier == 0.5 ? "SLOW (RB)" : "FAST (LB)");
                telemetry.addData("", "");
                telemetry.addData("Position", "X: %.1f, Y: %.1f", position.x, position.y);
                telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentState.heading));
                telemetry.addData("Speed", "%.1f in/sec", currentState.getSpeed());
                telemetry.addData("", "");
                telemetry.addData("Controls", "");
                telemetry.addData("Left Stick", "Drive/Strafe");
                telemetry.addData("Right Stick", "Turn");
                telemetry.addData("Start", "Reset heading");
                telemetry.update();
            }

            hardware.stopMotors();
        }
    }

