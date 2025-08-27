package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasHardware;

@TeleOp(name="Archytas: Hardware Test", group="Archytas")
public class ArchytasHardwareTest extends LinearOpMode {

    private ArchytasHardware hardware = new ArchytasHardware();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing hardware...");
        telemetry.update();

        // Initialize hardware
        boolean initSuccess = hardware.init(hardwareMap);

        if (!initSuccess) {
            telemetry.addData("ERROR", "Hardware initialization failed!");
            telemetry.addData("Check", "- Motor names in configuration");
            telemetry.addData("Check", "- IMU name and connection");
            telemetry.addData("Check", "- All devices are configured");
            telemetry.update();

            while (opModeIsActive()) {
                sleep(100);
            }
            return;
        }

        telemetry.addData("Status", "Hardware initialized successfully!");
        telemetry.addData("IMU Status", hardware.isIMUAvailable() ? "Available" : "Not Available");
        telemetry.addData("", "");
        telemetry.addData("Instructions", "");
        telemetry.addData("A", "Test individual motors");
        telemetry.addData("B", "Test drive forward");
        telemetry.addData("X", "Test strafe right");
        telemetry.addData("Y", "Manual drive test");
        telemetry.addData("DPAD_UP", "Reset IMU heading");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual drive test
            if (gamepad1.y) {
                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double[] powers = new double[4];
                powers[0] = drive + strafe + turn; // Front left
                powers[1] = drive - strafe - turn; // Front right
                powers[2] = drive - strafe + turn; // Back left
                powers[3] = drive + strafe - turn; // Back right

                // Normalize
                double max = Math.max(1.0, Math.max(
                        Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                        Math.max(Math.abs(powers[2]), Math.abs(powers[3]))
                ));

                for (int i = 0; i < 4; i++) {
                    powers[i] /= max;
                }

                hardware.setMotorPowers(powers);
            } else {
                hardware.stopMotors();
            }

            // Test buttons
            if (gamepad1.a) {
                telemetry.addData("Testing", "Individual motors...");
                telemetry.update();
                hardware.testMotors();
            }

            if (gamepad1.b) {
                telemetry.addData("Testing", "Drive forward...");
                telemetry.update();
                hardware.testDriveForward();
            }

            if (gamepad1.x) {
                telemetry.addData("Testing", "Strafe right...");
                telemetry.update();
                hardware.testStrafeRight();
            }

            if (gamepad1.dpad_up) {
                hardware.resetIMUHeading();
                telemetry.addData("Action", "IMU heading reset");
            }

            // Display telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("IMU Available", hardware.isIMUAvailable());
            telemetry.addData("IMU Heading", "%.2f degrees", Math.toDegrees(hardware.getIMUHeading()));
            telemetry.addData("", "");
            telemetry.addData("Encoders (inches)", "");
            telemetry.addData("Left", "%.2f", hardware.getLeftEncoderPosition());
            telemetry.addData("Right", "%.2f", hardware.getRightEncoderPosition());
            telemetry.addData("Perp", "%.2f", hardware.getPerpEncoderPosition());
            telemetry.addData("", "");
            telemetry.addData("Hold Y", "Manual drive test");
            telemetry.update();

            sleep(50);
        }

        hardware.stopMotors();
    }
}
