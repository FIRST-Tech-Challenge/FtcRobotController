package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.localization.EncoderTracking;
import org.firstinspires.ftc.teamcode.localization.Motion;
import org.firstinspires.ftc.teamcode.localization.Pose;

@Autonomous
public class Pose2PoseTest extends LinearOpMode {
    public static final double ACCEPT_DIST = 1.0; // inch. euclidean distance
    public static final double ACCEPT_TURN = degrees(5); // radian.
    public static final Motion.Calibrate CALIBRATION = new Motion.Calibrate(1.0, 1.0, 1.0);

    private static double degrees(double deg) {
        return deg * Math.PI / 180;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        EncoderTracking tracker = new EncoderTracking(hardware);
        Pose[] targets = {
                new Pose(24, 0, degrees(90)),
                new Pose(24, 24, degrees(180)),
                new Pose(0, 24, degrees(270)),
                new Pose(0, 0, degrees(0)),
        };
        int targetIndex = 0;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime targetTime = new ElapsedTime();
        waitForStart();
        targetTime.reset();
        timer.reset();
        boolean wait = false;
        while (opModeIsActive()) {
            tracker.step();
            Pose p = tracker.getPose();
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
                double linear = p.linearDistanceTo(targets[targetIndex]);
                double angular = p.subtractAngle(targets[targetIndex]);
                if (linear > ACCEPT_DIST || abs(angular) > ACCEPT_TURN) {
                    targetTime.reset();
                }
                if (targetTime.time() > 1.0) {
                    targetIndex++;
                    wait = true;
                    timer.reset();
                    continue;
                }
                double speed = min(max(0.5, linear / 18.0 + 0.1), timer.time() / 2);
                Motion action = p.to(targets[targetIndex], hardware);
                action.apply(hardware.driveMotors, CALIBRATION, speed);
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
            telemetry.update();
        }
        while (opModeIsActive()) {
            hardware.driveMotors.setAll(0);
            telemetry.addLine("done");
            telemetry.addData("x", tracker.getPose().x());
            telemetry.addData("y", tracker.getPose().y());
            telemetry.addData("heading (rad)", tracker.getPose().heading());
            telemetry.addData("heading (deg)", tracker.getPose().heading() * 180 / Math.PI);
            telemetry.update();
        }
    }
}
