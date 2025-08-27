package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasMovementAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasHardware;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.Point;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.RobotState;


@Autonomous(name="Archytas: Path Test", group="Archytas")
public class ArchytasPathTest extends LinearOpMode {

    private ArchytasHardware hardware = new ArchytasHardware();
    private ArchytasMovementAlgorithm movement = new ArchytasMovementAlgorithm();

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
        telemetry.addData("Path", "Square pattern (24x24 inches)");
        telemetry.update();

        waitForStart();

        // Reset everything
        hardware.resetEncoders();
        hardware.resetIMUHeading();
        movement.setPosition(new Point(0, 0));
        movement.setHeading(0);

        // Define a square path
        runSquarePath();

        telemetry.addData("Status", "Complete!");
        telemetry.update();

        while (opModeIsActive()) {
            sleep(100);
        }
    }

    private void runSquarePath() {
        // Square corners
        Point[] corners = {
                new Point(0, 0),      // Start
                new Point(24, 0),     // Right
                new Point(24, 24),    // Up
                new Point(0, 24),     // Left
                new Point(0, 0)       // Back to start
        };

        for (int i = 0; i < corners.length - 1; i++) {
            telemetry.addData("Status", "Moving to corner %d", i + 1);
            telemetry.update();

            // Move to next corner
            moveToPoint(corners[i + 1], 0); // 0 degrees heading

            if (!opModeIsActive()) break;
        }
    }

    private void moveToPoint(Point target, double targetHeading) {
        movement.setTarget(target);

        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();

        while (opModeIsActive() && !movement.isPathComplete() && timeout.seconds() < 10) {
            double deltaTime = hardware.getDeltaTime();

            // Update odometry
            movement.updateOdometry(
                    hardware.getLeftEncoderPosition(),
                    hardware.getRightEncoderPosition(),
                    hardware.getPerpEncoderPosition(),
                    hardware.getIMUHeading(),
                    deltaTime
            );

            // Get current state
            RobotState currentState = movement.getCurrentState();

            // Calculate motor powers
            double[] powers = movement.autonomousPointToPoint(currentState, targetHeading, deltaTime);
            hardware.setMotorPowers(powers);

            // Telemetry
            Point position = currentState.getPosition();
            double distance = position.distanceTo(target);

            telemetry.addData("Target", "X: %.1f, Y: %.1f", target.x, target.y);
            telemetry.addData("Current", "X: %.1f, Y: %.1f", position.x, position.y);
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentState.heading));
            telemetry.addData("Time", "%.1f seconds", timeout.seconds());
            telemetry.update();

            sleep(20);
        }

        hardware.stopMotors();
        movement.resetPath();
        sleep(500); // Pause between moves
    }
}

// =============================================================================
// PID TUNING OPMODE
// =============================================================================

