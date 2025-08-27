package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.Point;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.RobotState;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasMovementAlgorithm;
import org.firstinspires.ftc.teamcode.Subsystems.MovementAlgorithms.ArchytasHardware;


import java.util.Arrays;
import java.util.List;
@Autonomous(name="Archytas: Bezier Path Test", group="Archytas")
public class ArchytasBezierTest extends LinearOpMode {

    private ArchytasHardware hardware = new ArchytasHardware();
    private ArchytasMovementAlgorithm movement = new ArchytasMovementAlgorithm();

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
        telemetry.addData("Path", "S-curve using Bezier");
        telemetry.update();

        waitForStart();

        // Reset everything
        hardware.resetEncoders();
        hardware.resetIMUHeading();
        movement.setPosition(new Point(0, 0));
        movement.setHeading(0);

        // Run S-curve path
        runSCurvePath();

        telemetry.addData("Status", "Complete!");
        telemetry.update();

        while (opModeIsActive()) {
            sleep(100);
        }
    }

    private void runSCurvePath() {
        // Create an S-curve path
        List<Point> controlPoints = Arrays.asList(
                new Point(0, 0),      // Start
                new Point(12, 0),     // Control point 1
                new Point(12, 24),    // Control point 2
                new Point(24, 24),    // Control point 3
                new Point(36, 24),    // Control point 4
                new Point(36, 0),     // Control point 5
                new Point(48, 0)      // End
        );

        movement.setPath(controlPoints);

        ElapsedTime pathTime = new ElapsedTime();
        pathTime.reset();

        telemetry.addData("Status", "Following Bezier path...");
        telemetry.update();

        while (opModeIsActive() && !movement.isPathComplete() && pathTime.seconds() < 30) {
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

            // Calculate motor powers for path following
            double[] powers = movement.autonomousPathFollowing(currentState, deltaTime);
            hardware.setMotorPowers(powers);

            // Telemetry
            Point position = currentState.getPosition();
            Point target = controlPoints.get(controlPoints.size() - 1);
            double distanceToEnd = position.distanceTo(target);

            telemetry.addData("Status", "Following path...");
            telemetry.addData("Position", "X: %.1f, Y: %.1f", position.x, position.y);
            telemetry.addData("Target", "X: %.1f, Y: %.1f", target.x, target.y);
            telemetry.addData("Distance to End", "%.1f inches", distanceToEnd);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentState.heading));
            telemetry.addData("Speed", "%.1f in/sec", currentState.getSpeed());
            telemetry.addData("Time Elapsed", "%.1f seconds", pathTime.seconds());
            telemetry.addData("Path Progress", movement.isPathComplete() ? "COMPLETE" : "IN PROGRESS");
            telemetry.update();

            sleep(20);
        }

        hardware.stopMotors();

        if (pathTime.seconds() >= 30) {
            telemetry.addData("Status", "Timeout - Path incomplete");
        } else {
            telemetry.addData("Status", "Path completed successfully!");
        }
        telemetry.addData("Final Time", "%.1f seconds", pathTime.seconds());
        telemetry.update();
    }
}
