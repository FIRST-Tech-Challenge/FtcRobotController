package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.mmooover.Ramps;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.PoseFromToProcessor;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;

@Autonomous // Appear on the autonomous drop down
public class Pose2PoseTest extends LinearOpMode {
    public static final double ACCEPT_DIST = .5; // inch. euclidean distance
    public static final double ACCEPT_TURN = Math.toRadians(5);
    // power biases
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0); // Calibration factors for strafe, forward, and turn.

    @SuppressLint("DefaultLocale") // Android Lint should ignore warnings for DefaultLocale
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        EncoderTracking tracker = new EncoderTracking(hardware);
        // Pose targets to go thru
//        Pose[] targets = {
//                new Pose(48, 0, Math.toRadians(90)),
//                new Pose(48, 48, Math.toRadians(180)),
//                new Pose(0, 48, Math.toRadians(270)),
//                new Pose(0, 0, Math.toRadians(360))
//        };
        final Pose SCORE_HIGH_BASKET = new Pose(8, 19, Math.toRadians(-45));
        Pose[] targets = {
                SCORE_HIGH_BASKET,
                new Pose(28, 12, Math.toRadians(-180)),
                SCORE_HIGH_BASKET,
                new Pose(28, 22, Math.toRadians(-180)),
                SCORE_HIGH_BASKET,
                new Pose(65, 12, Math.toRadians(90)),
                new Pose(65, -8, Math.toRadians(90)),
//                new Pose(48, 0, Math.toRadians(90)),
//                new Pose(0, 0, Math.toRadians(0))
        };
        int targetIndex = 0; // Total poses in the set
        ElapsedTime timer = new ElapsedTime(); // Set timer object to reference the ElapsedTime object
        ElapsedTime targetTime = new ElapsedTime(); // Set targetTime to reference the ElapsedTime object
        LoopStopwatch ticker = new LoopStopwatch(); // Set ticker to reference the LoopStopwatch object
        // Set pftp to reference the PFTP object, set pose to (x, y, angle) --> (0, 0, 0)
        PoseFromToProcessor pftp = new PoseFromToProcessor(Pose.ORIGIN);
        Motion lastAction = null; // Store the previous action as empty (No actions have been recorded yet)
        Speed2Power speed2Power = new Speed2Power(0.20); // Set a speed2Power corresponding to a speed of 0.15 seconds
        Ramps ramps = new Ramps(
                Ramps.linear(2.0),
                Ramps.linear(1/12.0),
//                Easing.power(3.0, 12.0),
                Ramps.LimitMode.SCALE
        );

        telemetry.addLine("Initialized.");
        telemetry.update();

        waitForStart(); // Wait for start button

        telemetry.update();

        targetTime.reset(); // Restart times to prep for counting
        timer.reset();
        boolean wait = false;
        ticker.clear(); // Restart ticks

        while (opModeIsActive()) {
            ticker.click(); // Increment tick
            // Updates pose
            tracker.step();
            // Gets current pose
            Pose p = tracker.getPose();
            if (lastAction != null)
                pftp.update(lastAction.getPowerDifferential(), p);
            else
                pftp.update(0, p);
            if (wait) {
                hardware.driveMotors.setAll(0);
                if (targetIndex >= targets.length) {
                    targetTime.reset();
                    timer.reset();
                    break;
                }
                if (timer.time() > 1) {
                    wait = false;
                }
            } else {
                // Calculates the distance between current pose and target pose
                double linear = p.linearDistanceTo(targets[targetIndex]);
                double angular = p.subtractAngle(targets[targetIndex]);
                if (linear > ACCEPT_DIST || abs(angular) > ACCEPT_TURN) {
                    targetTime.reset();
                } else {
                    RobotLog.i("Getting to the target");
                }
                // Waits at the target for one second
                if (targetTime.time() > 1.0) {
                    RobotLog.i("Got to the target");
                    targetIndex++;
                    wait = true;
                    timer.reset();
                    continue;
                }
                // figure out how to get to the target position
                Motion action = pftp.getMotionToTarget(targets[targetIndex], hardware);
                double dToTarget = sqrt(
                        action.forward() * action.forward()
                                + action.right() * action.right()
                                + action.turn() * action.turn());
                double now = timer.time();
                double speed = ramps.ease(
                        now,
                        dToTarget,
                        0.75
                );
                action.apply(hardware.driveMotors, CALIBRATION, speed, speed2Power);
                telemetry.addData("forward", action.forward());
                telemetry.addData("right", action.right());
                telemetry.addData("turn (deg)", Math.toDegrees(action.turn()));
                String message = String.format(
                        "##%.3f##{\"step\":%d,\"pose\":[%.6f,%.6f,%.6f],\"motion\":[%.6f,%.6f,%.6f],\"speed\":%.6f," +
                                "\"frontLeft\":%.6f,\"frontRight\":%.6f,\"backLeft\":%.6f,\"backRight\":%.6f," +
                                "\"dToTarget\":%.6f,\"timer\":%.4f,\"avgTickTime\":%.6f}##",
                        System.currentTimeMillis() / 1000.0,
                        targetIndex,
                        p.x(), p.y(), p.heading(),
                        action.forward(), action.right(), action.turn(),
                        speed,
                        action.getLastFL(), action.getLastFR(), action.getLastBL(), action.getLastBR(),
                        dToTarget, now,
                        ticker.getAvg() * 1000
                );
                Log.d("DataDump", message);
                lastAction = action;
            }
            telemetry.addLine("step " + (targetIndex + 1) + " of " + targets.length);
            telemetry.addLine(String.format("Target hit for %.2fs", targetTime.time()));
            if (wait) {
                telemetry.addLine("wait...");
            } else {
                telemetry.addLine("go");
            }
            telemetry.addData("x", p.x()); // Print y attribute of pose
            telemetry.addData("y", p.y()); // Print x attribute of pose
            telemetry.addData("heading (rad)", p.heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(p.heading())); // Print the heading in degrees
            telemetry.addLine(); // Add space for format
            // Print the loop time for via ticks
            telemetry.addLine(String.format("Loop time: %.2fms", ticker.getAvg() * 1000));
            telemetry.update();
        }
        while (opModeIsActive()) {
            hardware.driveMotors.setAll(0);
            telemetry.addLine("done");
            telemetry.addData("x", tracker.getPose().x()); // Print x attribute for pose
            telemetry.addData("y", tracker.getPose().y()); // Print y attribute for pose
            telemetry.addData("heading (rad)", tracker.getPose().heading()); // Print the heading in radians
            telemetry.addData("heading (deg)", Math.toDegrees(tracker.getPose().heading())); // Print the heading in degrees
            telemetry.addLine(String.format("While running: %.2fms per loop", ticker.getAvg() * 1000));
            telemetry.update();
        }
        pftp.dump(); // Dump all values for Desmos graphing
    }
}
