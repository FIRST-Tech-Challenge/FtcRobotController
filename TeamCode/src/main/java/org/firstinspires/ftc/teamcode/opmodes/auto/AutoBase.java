package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * Shared setup and step helpers for autonomous routines.
 *
 * To write an auto: extend this class and fill in runRoutine() with steps like
 *     driveInches(24, RobotConstants.AUTO_POWER);
 *     turnToDegrees(90, RobotConstants.AUTO_POWER);
 *     shoot();
 *
 * Hive tipped check: robot.vision.isHiveTipped(minId, maxId), using one of the
 * RobotConstants.TAG_*_SCORING/AUDIENCE_MIN/MAX ranges for your alliance, then pick a plan.
 *
 * Safety: after AUTO_OVERDRIVE_SECONDS the remaining steps are skipped and
 * onOverdrive() runs instead (stop, or park).
 *
 * Alliance delay: set AUTO_START_DELAY_SECONDS in RobotConstants.
 * Time left: secondsLeft() tells a routine how much of the 30 seconds remains.
 *
 * TODO features to add:
 *  - Work out where we ended up (robot.odo pose) so we know how to get to park.
 *  - Park in the right zone: each auto gets a parkInZone() and overrides onOverdrive() to call it.
 *  - Use secondsLeft() to skip shooting and go park when time is short.
 */
public abstract class AutoBase extends LinearOpMode {

    protected Robot robot;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean overdriveRunning = false;
    private String lastShot = "none";

    /** Each auto puts its steps here. */
    protected abstract void runRoutine();

    /** Runs if time ran out before the routine finished. TODO: park. */
    protected void onOverdrive() {
        robot.drive.drive(0, 0, 0);
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
        robot.launcher.useAutoValues();

        waitForStart();
        matchTimer.reset();

        if (opModeIsActive()) {
            waitSeconds(RobotConstants.AUTO_START_DELAY_SECONDS); // alliance delay
            runRoutine();

            if (timeIsUp()) {
                overdriveRunning = true; // lets the steps in onOverdrive() run
                onOverdrive();
            }
        }

        robot.stop();
    }

    // ---- Steps ----

    /** Drive in a straight line. Positive power is forward, negative is backward. */
    protected void driveInches(double inches, double power) {
        robot.update();
        double startX = robot.odo.getXInches();
        double startY = robot.odo.getYInches();


        while (!shouldStop() && distanceFrom(startX, startY) < inches) {
            robot.drive.drive(power, 0, 0);
            tick();
        }
        stopAndPause();
    }

    /** Turn until the heading passes the target. 0 is straight ahead, positive is left. */
    protected void turnToDegrees(double targetDegrees, double power) {
        robot.update();
        boolean turnLeft = targetDegrees > robot.odo.getHeadingDegrees();

        while (!shouldStop() && (turnLeft
                ? robot.odo.getHeadingDegrees() < targetDegrees
                : robot.odo.getHeadingDegrees() > targetDegrees)) {
            robot.drive.drive(0, 0, turnLeft ? -power : power); // positive turn is clockwise
            tick();
        }
        stopAndPause();
    }

    /** Drive for a set time, for example to push into a wall. */
    protected void driveSeconds(double seconds, double power) {
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < seconds) {
            robot.drive.drive(power, 0, 0);
            tick();
        }
        stopAndPause();
    }

    /** Wait while the robot keeps updating (telemetry, odometry). */
    protected void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < seconds) {
            tick();
        }
    }

    /** Seconds left in the autonomous period. */
    protected double secondsLeft() {
        return RobotConstants.AUTO_PERIOD_SECONDS - matchTimer.seconds();
    }

    /**
     * Look for a tag between minId and maxId for up to LauncherConfig.SCAN_TIME seconds.
     * True means the hive is tipped.
     */
    protected boolean scanForTip(int minId, int maxId) {
        ElapsedTime timer = new ElapsedTime();
        while (!shouldStop() && timer.seconds() < LauncherConfig.SCAN_TIME) {
            if (robot.vision.isHiveTipped(minId, maxId)) return true;
            telemetry.addData("Scan", "%.1fs left", LauncherConfig.SCAN_TIME - timer.seconds());
            tick();
        }
        return false;
    }

    /**
     * Shoot all the balls. Spins up, waits once for the flywheel to be ready (or READY_TIMEOUT),
     * opens the gate with feed() and turns the intake on at the same time, then keeps going
     * for SHOOT_TIME so the balls launch.
     * Skipped if it's time to park. Turns the flywheel and intake off at the end.
     */
    protected void shoot() {
        if (timeToPark()) return;
        robot.launcher.spinUp(); // safe if already spinning

        ElapsedTime readyTimer = new ElapsedTime();
        while (!shouldStop() && !robot.launcher.isReady()
                && readyTimer.seconds() < LauncherConfig.READY_TIMEOUT) {
            tick();
        }
        lastShot = String.format("%s at %.0f, waited %.1fs",
                robot.launcher.isReady() ? "READY" : "TIMEOUT",
                robot.launcher.getVelocity(), readyTimer.seconds());

        robot.launcher.feed(); // opens the gate
        robot.intake.on();     // balls launch right away
        waitSeconds(LauncherConfig.SHOOT_TIME);

        robot.launcher.stopFlywheel();
        robot.launcher.retractFeeder();
        robot.intake.off();
    }

    // ---- Helpers ----

    private double distanceFrom(double startX, double startY) {
        return Math.hypot(robot.odo.getXInches() - startX, robot.odo.getYInches() - startY);
    }

    private boolean timeToPark() {
        return secondsLeft() <= LauncherConfig.PARK_TIME;
    }

    private boolean shouldStop() {
        return !opModeIsActive() || timeIsUp();
    }

    private boolean timeIsUp() {
        return !overdriveRunning && matchTimer.seconds() >= RobotConstants.AUTO_OVERDRIVE_SECONDS;
    }

    /** One pass of the loop: show the match time and let every subsystem update. */
    private void tick() {
        telemetry.addData("Match time", "%.1f (%.1f left)", matchTimer.seconds(), secondsLeft());
        telemetry.addData("Last shot", lastShot);
        robot.update();
    }

    private void stopAndPause() {
        robot.drive.drive(0, 0, 0);
        robot.update();
        sleep(RobotConstants.AUTO_PAUSE_MS);
    }
}
