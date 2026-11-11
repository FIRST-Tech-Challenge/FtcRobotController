package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

/**
 * One-gamepad tool for building the distance -> launcher velocity table.
 *
 * At each spot: aim, find the lowest and highest velocities that still score, set the velocity
 * halfway between, and press X to record (distance, velocity). Recorded points show on telemetry
 * and are also written to the robot log with the tag "LauncherCal".
 *
 * Load balls by hand (no intake). Line the turret up facing straight forward before INIT.
 */
@TeleOp(name = "Launcher Calibration", group = "Test")
public class LauncherCalibration extends GreenBotBaseOpMode {

    private static final double START_VELOCITY = 1250;
    private static final double BIG_STEP = 50;     // D-pad up/down
    private static final double SMALL_STEP = 10;   // D-pad right/left

    private static final double DRIVE_SPEED = 0.3; // slow, for repositioning between spots
    private static final double TURRET_NUDGE_POWER = 0.25;
    private static final double FEEDER_POWER = 1.0;

    private static final int DISTANCE_SAMPLES = 20; // readings averaged for the distance shown
    private static final int POINTS_SHOWN = 8;      // most recent recorded points on screen

    private double velocity = START_VELOCITY;
    private boolean launcherOn = true;

    private final ArrayDeque<Double> recentDistances = new ArrayDeque<>();
    private final List<String> points = new ArrayList<>();
    private String message = "";

    @Override
    public void runOpMode() {
        config();
        aimer.resetTurretEncoder(); // turret should face straight forward now

        while (opModeInInit()) {
            if (gamepad1.x) aimer.setBlueAlliance(true);
            if (gamepad1.b) aimer.setBlueAlliance(false);
            telemetry.addData("Alliance", "%s   (X = BLUE, B = RED)",
                    aimer.isBlueAlliance() ? "BLUE" : "RED");
            telemetry.addLine("Turret encoder zeroed -- it should be facing straight forward.");
            telemetry.update();
        }

        waitForStart();
        gamepad1.xWasPressed(); // forget X presses from alliance picking so they don't record
        wedgeServo.setPosition(WEDGE_UP);
        aimer.start();

        while (opModeIsActive()) {
            // ---------- Drive (slow) ----------
            drive.drive(
                    -gamepad1.left_stick_y * DRIVE_SPEED,
                    gamepad1.left_stick_x * DRIVE_SPEED,
                    -gamepad1.right_stick_x * DRIVE_SPEED);

            // ---------- Turret: hold Y to auto-aim, bumpers to nudge ----------
            aimer.update();
            if (gamepad1.left_bumper) {
                aimer.moveTurret(-TURRET_NUDGE_POWER);
            } else if (gamepad1.right_bumper) {
                aimer.moveTurret(TURRET_NUDGE_POWER);
            } else if (gamepad1.y) {
                aimer.aim();
            } else {
                aimer.stopTurret();
            }

            // ---------- Launcher: A toggles, D-pad sets velocity ----------
            if (gamepad1.aWasPressed()) launcherOn = !launcherOn;
            if (gamepad1.dpadUpWasPressed()) velocity += BIG_STEP;
            if (gamepad1.dpadDownWasPressed()) velocity -= BIG_STEP;
            if (gamepad1.dpadRightWasPressed()) velocity += SMALL_STEP;
            if (gamepad1.dpadLeftWasPressed()) velocity -= SMALL_STEP;
            velocity = Range.clip(velocity,
                    Launcher.MIN_TARGET_VELOCITY, Launcher.MAX_TARGET_VELOCITY);

            if (launcherOn) launcher.setTargetVelocity(velocity);
            else launcher.stop();

            // ---------- Feeder: hold right trigger to shoot, hold B to back out ----------
            if (gamepad1.right_trigger > 0.5) feeder.setPower(FEEDER_POWER);
            else if (gamepad1.b) feeder.setPower(-FEEDER_POWER);
            else feeder.setPower(0);

            updateLights();

            // ---------- Distance: average recent readings so the number holds still ----------
            if (aimer.hasTarget()) {
                recentDistances.addLast(aimer.getDistanceInches());
                if (recentDistances.size() > DISTANCE_SAMPLES) recentDistances.removeFirst();
            } else {
                recentDistances.clear(); // don't let old readings get recorded
            }
            double distance = averageDistance();

            // ---------- X records a data point ----------
            if (gamepad1.xWasPressed()) {
                if (recentDistances.isEmpty()) {
                    message = "NOT RECORDED - no hive in view";
                } else {
                    String point = String.format("%.1f in -> %.0f", distance, velocity);
                    points.add(point);
                    RobotLog.ii("LauncherCal", "point %d: %s", points.size(), point);
                    message = "Recorded #" + points.size() + ": " + point;
                }
            }

            showTelemetry(distance);
        }

        drive.stopDriving();
        aimer.stop();
        launcher.stop();
        feeder.setPower(0);
    }

    private double averageDistance() {
        if (recentDistances.isEmpty()) return 0;
        double sum = 0;
        for (double d : recentDistances) sum += d;
        return sum / recentDistances.size();
    }

    private void showTelemetry(double distance) {
        if (recentDistances.isEmpty()) {
            telemetry.addData("Distance", "NO HIVE IN VIEW");
        } else {
            telemetry.addData("Distance (in)", "%.1f   (raw %.1f, %d tags)",
                    distance, aimer.getDistanceInches(), aimer.getTagCount());
        }
        telemetry.addData("Velocity", "%.0f %s", velocity, launcherOn ? "" : "(launcher OFF)");
        launcher.showStatus();
        telemetry.addData("Aim", "%s   (tx %.2f deg)",
                aimer.isAimed() ? "LINED UP" : (aimer.hasTarget() ? "not lined up" : "no hive"),
                aimer.getTxDegrees());
        telemetry.addData("Turret", aimer.getStatus());
        if (!message.isEmpty()) telemetry.addLine(message);

        telemetry.addLine();
        telemetry.addLine("Recorded points (distance -> velocity):");
        for (int i = Math.max(0, points.size() - POINTS_SHOWN); i < points.size(); i++) {
            telemetry.addLine("  " + (i + 1) + ".  " + points.get(i));
        }

        telemetry.addLine();
        telemetry.addLine("Sticks: drive (slow)   Y (hold): auto-aim   Bumpers: turret");
        telemetry.addLine(String.format("D-pad Up/Down: +/-%.0f   Right/Left: +/-%.0f   A: launcher on/off",
                BIG_STEP, SMALL_STEP));
        telemetry.addLine("Right trigger (hold): feed   B (hold): feeder reverse   X: record point");
        telemetry.update();
    }
}
