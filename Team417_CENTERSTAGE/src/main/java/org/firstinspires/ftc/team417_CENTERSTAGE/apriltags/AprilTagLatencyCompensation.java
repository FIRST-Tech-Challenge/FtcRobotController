package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseOpMode;
import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.TwistWithTimestamp;

public class AprilTagLatencyCompensation {
    public static void compensateForLatency(MecanumDrive mecanumDrive, Twist2dDual<Time> twist) {
        // From now on to "END", everything is added by Hank
        // To keep a record of twists to be used by April Tag latency compensation
        if (mecanumDrive.USE_APRIL_TAGS) {
            mecanumDrive.twistList.add(0, new TwistWithTimestamp(twist, mecanumDrive.clock.milliseconds()));

            // Keep only twists from less than five seconds ago
            if (mecanumDrive.twistList.size() > 0) {
                TwistWithTimestamp oldestTwist = mecanumDrive.twistList.get(mecanumDrive.twistList.size() - 1);
                double currentTime = mecanumDrive.clock.milliseconds();
                while (oldestTwist.timestamp < currentTime - 5000) {
                    mecanumDrive.twistList.remove(oldestTwist);
                    oldestTwist = mecanumDrive.twistList.get(mecanumDrive.twistList.size() - 1);
                    currentTime = mecanumDrive.clock.milliseconds();
                }
            }

            mecanumDrive.myAprilTagPoseEstimator.updatePoseEstimate();

            Pose2d poseEstimation = mecanumDrive.myAprilTagPoseEstimator.estimatePose();

            boolean anyMotorRunning = false;
            for (DcMotorEx motor : mecanumDrive.motors) {
                if (!BaseOpMode.isEpsilonEquals(motor.getPower(), 0)) {
                    anyMotorRunning = true;
                    break;
                }
            }

            if (poseEstimation != null && mecanumDrive.twistList.size() > 1 && !anyMotorRunning) {
                System.out.println("USE APRIL TAGS!");

                if (mecanumDrive.isDevBot) {
                    mecanumDrive.myAprilTagPoseEstimator.statusLight.setState(false);
                }
                mecanumDrive.pose = poseEstimation;

                // Latency compensation code (add only when thoroughly tested)
                double currentTime = mecanumDrive.clock.milliseconds();
                TwistWithTimestamp lastTwist = mecanumDrive.twistList.get(0);
                for (int i = 1; lastTwist.timestamp >= currentTime - mecanumDrive.myAprilTagPoseEstimator.CAMERA_LATENCY && i < mecanumDrive.twistList.size(); i++) {
                    mecanumDrive.pose.plus(lastTwist.twist.value());
                    lastTwist = mecanumDrive.twistList.get(i);
                    currentTime = mecanumDrive.clock.milliseconds();
                }

            } else {
                System.out.println("NOT USE APRIL TAGS!");

                if (mecanumDrive.isDevBot) {
                    mecanumDrive.myAprilTagPoseEstimator.statusLight.setState(true);
                }
                System.out.println(twist.value().toString());
                mecanumDrive.pose = mecanumDrive.pose.plus(twist.value()); // This line was actually in the original MecanumDrive class, just moved here by me
            }
        } else {
            mecanumDrive.pose = mecanumDrive.pose.plus(twist.value()); // This line was actually in the original MecanumDrive class, just moved here by me
        }
    }
}
