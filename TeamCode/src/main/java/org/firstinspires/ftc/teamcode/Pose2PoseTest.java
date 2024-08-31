package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mmooover.Easing;
import org.firstinspires.ftc.teamcode.mmooover.EncoderTracking;
import org.firstinspires.ftc.teamcode.mmooover.Motion;
import org.firstinspires.ftc.teamcode.mmooover.Pose;
import org.firstinspires.ftc.teamcode.mmooover.PoseFromToProcessor;
import org.firstinspires.ftc.teamcode.mmooover.Speed2Power;
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch;

@Autonomous // Appear on the autonomous drop down
public class Pose2PoseTest extends LinearOpMode {
    public static final double ACCEPT_DIST = 1.0; // inch. euclidean distance
    public static final double ACCEPT_TURN = deg2rad(5);
    // power biases
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0); // Calibration factors for strafe, forward, and turn.

    private static double deg2rad(double deg) {
        return deg * Math.PI / 180;
    }

    @SuppressLint("DefaultLocale") // Android Lint should ignore warnings for DefaultLocale
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        EncoderTracking tracker = new EncoderTracking(hardware);
        // Pose targets to go thru
        Pose[] targets = {
                new Pose(48, 0, deg2rad(180))
        };
        int targetIndex = 0; // Total poses in the set
        ElapsedTime timer = new ElapsedTime(); // Set timer object to reference the ElapsedTime object
        ElapsedTime targetTime = new ElapsedTime(); // Set targetTime to reference the ElapsedTime object
        LoopStopwatch ticker = new LoopStopwatch(); // Set ticker to reference the LoopStopwatch object
        PoseFromToProcessor pftp = new PoseFromToProcessor(Pose.ORIGIN); // Set pftp to reference the PFTP object, set pose to (x, y, angle) --> (0, 0, 0)
        Motion lastAction = null; // Store the previous action as empty (No actions have been recorded yet)
        Speed2Power speed2Power = new Speed2Power(0.15); // Set a speed2Power corresponding to a speed of 0.15 seconds
        Easing easingFunction = new Easing(
                Easing.linear(2.0),
                Easing.linear(1/12.0),
//                Easing.power(3.0, 12.0),
                Easing.LimitMode.SCALE
        );

        waitForStart(); // Wait for start button

        targetTime.reset();
        timer.reset();
        boolean wait = false;
        ticker.clear();

        while (opModeIsActive()) {
            ticker.click();
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
                }
                // Waits at the target for one second
                if (targetTime.time() > 1.0) {
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
                double speed = easingFunction.ease(
                        timer.time(),
                        dToTarget,
                        0.75
                );
                double power = speed2Power.speed2power(speed);
                action.apply(hardware.driveMotors, CALIBRATION, power);
                lastAction = action;
            }
            telemetry.addLine("step " + (targetIndex + 1) + " of " + targets.length);
            telemetry.addLine(String.format("Target hit for %.2fs", targetTime.time()));
            if (wait) {
                telemetry.addLine("wait...");
            } else {
                telemetry.addLine("go");
            }
            telemetry.addData("x", p.x());
            telemetry.addData("y", p.y());
            telemetry.addData("heading (rad)", p.heading());
            telemetry.addData("heading (deg)", p.heading() * 180 / Math.PI);
            telemetry.addLine();
            telemetry.addLine(String.format("Loop time: %.2fms", ticker.getAvg() * 1000));
            telemetry.update();
        }
        while (opModeIsActive()) {
            hardware.driveMotors.setAll(0);
            telemetry.addLine("done");
            telemetry.addData("x", tracker.getPose().x());
            telemetry.addData("y", tracker.getPose().y());
            telemetry.addData("heading (rad)", tracker.getPose().heading());
            telemetry.addData("heading (deg)", tracker.getPose().heading() * 180 / Math.PI);
            telemetry.addLine(String.format("While running: %.2fms per loop", ticker.getAvg() * 1000));
            telemetry.update();
        }
        pftp.dump(); // Dump all values for Desmos graphing
    }
}
