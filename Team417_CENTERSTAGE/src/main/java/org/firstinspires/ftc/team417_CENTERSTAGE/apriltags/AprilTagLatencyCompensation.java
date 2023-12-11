package org.firstinspires.ftc.team417_CENTERSTAGE.apriltags;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417_CENTERSTAGE.utilityclasses.TwistWithTimestamp;

public class AprilTagLatencyCompensation {

    // Todo: only run method when having new April tag frame

    public static void compensateForLatency(MecanumDrive mecanumDrive, Twist2dDual<Time> twist) {
        // To keep a record of twists to be used by April Tag latency compensation
        if (mecanumDrive.USE_APRIL_TAGS) {
            mecanumDrive.twistList.add(0, new TwistWithTimestamp(twist, mecanumDrive.clock.milliseconds()));

            // Todo: change 5000 ms to (latency time + frame rate)
            // Keep only twists from less than five seconds ago
            if (mecanumDrive.twistList.size() > 0) {
                TwistWithTimestamp oldestTwist = mecanumDrive.twistList.get(mecanumDrive.twistList.size() - 1);
                double currentTime = mecanumDrive.clock.milliseconds();
                while (oldestTwist.timestamp < currentTime - mecanumDrive.myAprilTagPoseEstimator.CAMERA_LATENCY
                        + (1000 / mecanumDrive.myAprilTagPoseEstimator.visionPortal.getFps())) {
                    mecanumDrive.twistList.remove(oldestTwist);
                    oldestTwist = mecanumDrive.twistList.get(mecanumDrive.twistList.size() - 1);
                    currentTime = mecanumDrive.clock.milliseconds();
                }
            }

            mecanumDrive.myAprilTagPoseEstimator.updatePoseEstimate();

            Pose2d poseEstimation = mecanumDrive.myAprilTagPoseEstimator.estimatePose();


            if (poseEstimation != null && mecanumDrive.twistList.size() > 1) {
                if (mecanumDrive.isDevBot) {
                    mecanumDrive.myAprilTagPoseEstimator.statusLight.setState(false);
                }
                mecanumDrive.pose = poseEstimation;

                // Latency compensation code
                double currentTime = mecanumDrive.clock.milliseconds();

                // Get the first twist needed to be added the robot's position
                TwistWithTimestamp lastTwist = mecanumDrive.twistList.get(0);

                // Add all the twists since the April Tag frame to now to the robot pose
                for (int i = 1; lastTwist.timestamp >= currentTime - mecanumDrive.myAprilTagPoseEstimator.CAMERA_LATENCY && i < mecanumDrive.twistList.size(); i++) {
                    mecanumDrive.pose.plus(lastTwist.twist.value());
                    lastTwist = mecanumDrive.twistList.get(i);
                    currentTime = mecanumDrive.clock.milliseconds();
                }

            } else {
                if (mecanumDrive.isDevBot) {
                    mecanumDrive.myAprilTagPoseEstimator.statusLight.setState(true);
                }
                mecanumDrive.pose = mecanumDrive.pose.plus(twist.value()); // This line was actually in the original MecanumDrive class, just moved here
            }
        } else {
            mecanumDrive.pose = mecanumDrive.pose.plus(twist.value()); // This line was actually in the original MecanumDrive class, just moved here
        }
    }
}
