package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

/**
 * Runs Drive.driveTo() through every kind of move it has to handle, one step at a time, so you
 * can watch each one and read how close it got.
 *
 * Controls (gamepad 1), between steps:
 *   A = run the next step
 *   B = go back to where the last step started and run it again
 *   Y = run all remaining steps without stopping
 *   X = toggle between bang-bang and PID driving (also works during INIT)
 *
 * To compare the two modes on one move: run it, press X, then press B to run it again.
 *
 * Every target is an offset from the start pose, so the whole test fits in a box from the start
 * spot to 36" in front of the robot and 12" to either side. Place the robot with its back to
 * the wall, facing START_HEADING, with that much clear floor in front of it.
 *
 * Robot facing 180 (toward -x): robot-forward is -x, robot-left is -y, robot-right is +y.
 */
@Autonomous(name = "Auton Drive Test", group = "Test")
public class AutonDriveTest extends GreenBotBaseOpMode {

    static final double START_X = 63;
    static final double START_Y = -12;
    static final double START_HEADING = 180;

    static final double SLOW = 0.3;
    static final double MED = 0.5;
    static final double FAST = 1.0;
    static final double TURN = 0.5;
    static final double TIMEOUT = 6.0; // seconds per step
    static final long SETTLE_PAUSE_MS = 300; // wait after each step before measuring

    /** One move: a name, a target offset from the start pose, and how hard to push. */
    static class Step {
        final String name;
        final double dx, dy, heading, power, turnPower;

        Step(String name, double dx, double dy, double heading, double power, double turnPower) {
            this.name = name;
            this.dx = dx;
            this.dy = dy;
            this.heading = heading;
            this.power = power;
            this.turnPower = turnPower;
        }

        Step(String name, double dx, double dy, double heading) {
            this(name, dx, dy, heading, SLOW, TURN);
        }
    }

    static final Step[] STEPS = {
            // --- Straight lines, robot square to the field ---
            new Step("Forward 24",              -24,   0, 180),
            new Step("Backward 12",             -12,   0, 180),
            new Step("Strafe left 12",          -12, -12, 180),
            new Step("Strafe right 24",         -12,  12, 180),
            new Step("Strafe left 12 (center)", -12,   0, 180),

            // --- 45-degree diagonals, all four directions ---
            new Step("Diagonal fwd-left",       -24, -12, 180),
            new Step("Diagonal back-right",     -12,   0, 180),
            new Step("Diagonal fwd-right",      -24,  12, 180),
            new Step("Diagonal back-left",      -12,   0, 180),

            // --- A shallow (non-45) diagonal ---
            new Step("Shallow diag fwd-left",   -36,  -6, 180),
            new Step("Shallow diag back-right", -12,   0, 180),

            // --- Turns in place: both directions, small and large, across the +/-180 seam ---
            new Step("Turn CW 90 (to 90)",      -12,   0,   90),
            new Step("Turn CW 90 (to 0)",       -12,   0,    0),
            new Step("Turn CCW 135 (to 135)",   -12,   0,  135),
            new Step("Turn CCW 75 across 180",  -12,   0, -150),
            new Step("Turn CW 30 back to 180",  -12,   0,  180),
            new Step("Tiny turn 8 deg",         -12,   0,  172),
            new Step("Tiny turn back",          -12,   0,  180),

            // --- Moving while held at an odd angle (field-relative math) ---
            new Step("Turn to odd angle 210",   -12,   0, -150),
            new Step("At 210: field -x 12",     -24,   0, -150),
            new Step("At 210: field -y 12",     -24, -12, -150),
            new Step("At 210: diagonal back",   -12,   0, -150),
            new Step("Turn to odd angle 150",   -12,   0,  150),
            new Step("At 150: diag fwd-right",  -24,  12,  150),
            new Step("At 150: field -y 12",     -24,   0,  150),
            new Step("At 150: field +x 12",     -12,   0,  150),
            new Step("Square up to 180",        -12,   0,  180),

            // --- Turning while moving ---
            new Step("Fwd 24 + turn to 90",     -36,   0,   90),
            new Step("Back 24 + turn to 180",   -12,   0,  180),
            new Step("Diag + turn to 225",      -24, -12, -135),
            new Step("Diag + turn to 135",      -12,  12,  135),
            new Step("Strafe + turn to 180",    -12,   0,  180),

            // --- Precision: very short moves ---
            new Step("Short fwd 2",             -14,   0, 180),
            new Step("Short left 2",            -14,  -2, 180),
            new Step("Short back to center",    -12,   0, 180),

            // --- Speed: same move at increasing power, watch the overshoot ---
            new Step("Fwd 24 @ 0.5",            -36,   0, 180, MED,  TURN),
            new Step("Back 24 @ 0.5",           -12,   0, 180, MED,  TURN),
            new Step("Fwd 24 @ 1.0",            -36,   0, 180, FAST, TURN),
            new Step("Back 24 @ 1.0",           -12,   0, 180, FAST, TURN),

            // --- Home ---
            new Step("Return to start",           0,   0, 180),
    };

    @Override
    public void runOpMode() {
        config();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) drive.setUsePid(!drive.isUsingPid());
            telemetry.addData("Ready", "%d steps. Press START, then A for each step.",
                    STEPS.length);
            telemetry.addData("Mode", "%s   (X to toggle)", modeName());
            drive.showPose();
            telemetry.update();
        }

        boolean runAll = false;
        int i = 0;
        String lastResult = "(none yet)";

        while (opModeIsActive() && i < STEPS.length) {
            Step step = STEPS[i];

            // Wait for the driver unless we are running straight through.
            boolean repeat = false;
            while (!runAll && opModeIsActive()) {
                telemetry.addData("Last", lastResult);
                telemetry.addData("Next", "%d/%d  %s", i + 1, STEPS.length, step.name);
                telemetry.addData("Mode", modeName());
                telemetry.addData("Controls", "A next   B repeat last   Y run all   X toggle PID");
                drive.showPose();
                telemetry.update();

                if (gamepad1.xWasPressed()) drive.setUsePid(!drive.isUsingPid());
                if (gamepad1.aWasPressed()) break;
                if (gamepad1.yWasPressed()) { runAll = true; break; }
                if (gamepad1.bWasPressed() && i > 0) { repeat = true; break; }
            }
            if (!opModeIsActive()) break;
            if (repeat) {
                // The robot is sitting at the end of the last step, so first go back to where
                // that step started (the step before it, or the start pose) and then rerun it.
                step = STEPS[--i];
                telemetry.addData("Repeat", "Returning to the start of: %s", step.name);
                telemetry.update();
                if (i > 0) {
                    Step before = STEPS[i - 1];
                    drive.driveTo(START_X + before.dx, START_Y + before.dy, before.heading,
                            SLOW, TURN, TIMEOUT);
                } else {
                    drive.driveTo(START_X, START_Y, START_HEADING, SLOW, TURN, TIMEOUT);
                }
                if (!opModeIsActive()) break;
                sleep(SETTLE_PAUSE_MS);
            }

            lastResult = runStep(i, step);
            i++;
        }

        // Leave the final result on screen until STOP.
        while (opModeIsActive()) {
            telemetry.addData("Done", "All steps finished");
            telemetry.addData("Last", lastResult);
            drive.showPose();
            telemetry.update();
        }
    }

    /** Drives one step and returns a one-line summary of how it went. */
    private String runStep(int index, Step step) {
        double targetX = START_X + step.dx;
        double targetY = START_Y + step.dy;

        ElapsedTime timer = new ElapsedTime();
        drive.driveTo(targetX, targetY, step.heading, step.power, step.turnPower, TIMEOUT);
        double seconds = timer.seconds();

        // Let the robot finish coasting before measuring, so a mode that stops early and
        // drifts gets judged on where it actually ends up. Not counted in the time.
        sleep(SETTLE_PAUSE_MS);

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double errX = targetX - pose.getX(DistanceUnit.INCH);
        double errY = targetY - pose.getY(DistanceUnit.INCH);
        double errH = AngleUnit.normalizeDegrees(step.heading - pose.getHeading(AngleUnit.DEGREES));

        return String.format("%d %s [%s]: off %.1f in (x %.1f, y %.1f), %.1f deg, %.1fs%s",
                index + 1, step.name, modeName(), Math.hypot(errX, errY), errX, errY, errH,
                seconds, seconds >= TIMEOUT ? "  TIMED OUT" : "");
    }

    private String modeName() {
        return drive.isUsingPid() ? "PID" : "Bang-bang";
    }
}
