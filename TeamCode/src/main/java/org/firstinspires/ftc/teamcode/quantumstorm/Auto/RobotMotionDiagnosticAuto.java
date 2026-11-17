package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;

/**
 * Open-floor diagnostic for drive direction, strafe direction, and Pinpoint heading.
 * It deliberately does not run shooter, feeder, intake, Limelight, or match routing.
 */
@Autonomous(name = "TEST Robot Motion Sequence", group = "Testing")
public final class RobotMotionDiagnosticAuto extends LinearOpMode {
    private static final double MOVE_TOLERANCE_IN = 1.0;
    private static final double TURN_TOLERANCE_DEG = 2.5;
    private static final double PAUSE_SECONDS = 1.0;

    private enum Kind { MOVE, TURN }
    private static final class Step {
        final String label;
        final Kind kind;
        final double amount;
        Step(String label, Kind kind, double amount) { this.label=label; this.kind=kind; this.amount=amount; }
    }
    private static final Step[] STEPS = {
            new Step("Drive forward 12 inches", Kind.MOVE, 12),
            new Step("Strafe right 12 inches", Kind.MOVE, 12),
            new Step("Turn left 30 degrees", Kind.TURN, 30),
            new Step("Turn right 30 degrees", Kind.TURN, -30),
            new Step("Turn left 90 degrees", Kind.TURN, 90),
            new Step("Turn right 90 degrees", Kind.TURN, -90),
            new Step("Turn left 180 degrees", Kind.TURN, 180),
            new Step("Turn right 180 degrees", Kind.TURN, -180),
            new Step("Turn left 360 degrees", Kind.TURN, 360),
            new Step("Turn right 360 degrees", Kind.TURN, -360)
    };

    private static double wrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
    private static double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    @Override public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap,
                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        ElapsedTime clock = new ElapsedTime();
        String fault = "";

        while (opModeInInit() && !isStopRequested()) {
            odometry.update();
            telemetry.addLine("TEST ONLY: place robot in an open area with at least 4 ft clearance.");
            telemetry.addLine("Sequence: forward 12, right 12, L/R 30, L/R 90, L/R 180, L/R 360.");
            telemetry.addLine("Every step stops for one second. Press STOP at any time.");
            odometry.addTelemetry(telemetry);
            telemetry.update();
            sleep(50);
        }
        waitForStart();
        if (isStopRequested()) return;

        int stepIndex = 0;
        boolean pausing = false;
        double phaseStarted = -1;
        double startX = 0, startY = 0, startHeading = 0;
        double previousHeading = 0, accumulatedTurn = 0;

        try {
            while (opModeIsActive() && stepIndex < STEPS.length) {
                odometry.update();
                if (!odometry.isValid()) {
                    fault = "Pinpoint is not READY; motion stopped.";
                    break;
                }

                double x = odometry.getX();
                double y = odometry.getY();
                double heading = Math.toRadians(odometry.getHeading());
                if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
                    fault = "Pinpoint returned an invalid pose; motion stopped.";
                    break;
                }

                Step step = STEPS[stepIndex];
                if (phaseStarted < 0) {
                    phaseStarted = clock.seconds();
                    startX = x; startY = y; startHeading = heading;
                    previousHeading = heading; accumulatedTurn = 0;
                }

                double elapsed = clock.seconds() - phaseStarted;
                boolean complete;
                if (pausing) {
                    drive.stop();
                    complete = elapsed >= PAUSE_SECONDS;
                    telemetry.addData("Pause after", step.label);
                    telemetry.addData("Pause remaining", "%.1f s", Math.max(0, PAUSE_SECONDS - elapsed));
                } else if (step.kind == Kind.MOVE) {
                    // First movement is forward; second movement is robot-right.
                    double forwardTarget = stepIndex == 0 ? step.amount : 0;
                    double rightTarget = stepIndex == 1 ? step.amount : 0;
                    double dx = x - startX, dy = y - startY;
                    double forward = dx * Math.cos(startHeading) + dy * Math.sin(startHeading);
                    double right = dx * Math.sin(startHeading) - dy * Math.cos(startHeading);
                    double forwardError = forwardTarget - forward;
                    double rightError = rightTarget - right;
                    complete = Math.abs(forwardError) <= MOVE_TOLERANCE_IN && Math.abs(rightError) <= MOVE_TOLERANCE_IN;
                    if (!complete) {
                        double holdHeading = wrap(startHeading - heading);
                        // MecanumDrive has positive forward. AUTO_FORWARD_POWER_SIGN
                        // matches this diagnostic to the autonomous drivetrain setup.
                        drive.drive(clip(forwardError * .10, .35) * Constants.AUTO_FORWARD_POWER_SIGN,
                                clip(rightError * .10, .35), clip(-holdHeading * 1.2, .25));
                    } else drive.stop();
                    telemetry.addData("Move progress", "forward %.1f / %.1f, right %.1f / %.1f in",
                            forward, forwardTarget, right, rightTarget);
                } else {
                    accumulatedTurn += wrap(heading - previousHeading);
                    previousHeading = heading;
                    double targetRadians = Math.toRadians(step.amount);
                    double error = targetRadians - accumulatedTurn;
                    complete = Math.abs(Math.toDegrees(error)) <= TURN_TOLERANCE_DEG;
                    if (!complete) drive.drive(0, 0, clip(-error * .75, .35));
                    else drive.stop();
                    telemetry.addData("Turn progress", "%.1f / %.1f deg", Math.toDegrees(accumulatedTurn), step.amount);
                }

                double timeout = step.kind == Kind.MOVE ? 7 : Math.max(5, Math.abs(step.amount) / 45.0 + 3);
                if (!pausing && elapsed >= timeout) {
                    drive.stop();
                    fault = "Timed out on " + step.label + ". Check drive direction and Pinpoint axes.";
                    break;
                }
                if (complete) {
                    drive.stop();
                    if (pausing) { stepIndex++; pausing=false; phaseStarted=-1; }
                    else { pausing=true; phaseStarted=clock.seconds(); }
                }

                telemetry.addData("Step", "%d / %d: %s", stepIndex + 1, STEPS.length, step.label);
                telemetry.addData("State", pausing ? "STOPPED - one-second pause" : "MOVING");
                odometry.addTelemetry(telemetry);
                drive.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
        } finally {
            drive.stop();
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Motion test", fault.isEmpty() ? "COMPLETE - drive is stopped" : "STOPPED");
            if (!fault.isEmpty()) telemetry.addData("Reason", fault);
            telemetry.addLine("Press STOP to exit.");
            odometry.addTelemetry(telemetry);
            telemetry.update();
            idle();
        }
    }
}
