package org.firstinspires.ftc.teamcode;

import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Waypoint;
import com.acmerobotics.splinelib.control.PIDCoefficients;
import com.acmerobotics.splinelib.followers.GVFFollower;
import com.acmerobotics.splinelib.followers.MecanumPIDVAFollower;
import com.acmerobotics.splinelib.path.Path;
import com.acmerobotics.splinelib.path.QuinticSplineSegment;
import com.acmerobotics.splinelib.trajectory.DriveConstraints;
import com.acmerobotics.splinelib.trajectory.MecanumConstraints;
import com.acmerobotics.splinelib.trajectory.PathTrajectorySegment;
import com.acmerobotics.splinelib.trajectory.TankConstraints;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.util.Arrays;
import java.util.Collections;

@Autonomous
public class SplineFollowTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyMecanumDrive drive = new MyMecanumDrive(hardwareMap);
        Path path = new Path(new QuinticSplineSegment(
                new Waypoint(0.0, 0.0, 60.0, 0.0),
                new Waypoint(40.0, 40.0, 60.0, 0.0)
        ));
        DriveConstraints baseConstraints = new DriveConstraints(20.0, 30.0, Math.PI / 2, Math.PI / 2);
        MecanumConstraints constraints = new MecanumConstraints(baseConstraints, drive.getTrackWidth(), drive.getWheelBase());
        Trajectory trajectory = new Trajectory(Collections.singletonList(new PathTrajectorySegment(path, constraints, 250)));
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                new PIDCoefficients(0, 0, 0),
                new PIDCoefficients(0, 0, 0),
                0.03631,
                0,
                0);
        CSVWriter writer = new CSVWriter(new File(LoggingUtil.getLogRoot(this), "SplineFollowTest-" + System.currentTimeMillis() + ".csv"));

        waitForStart();

        double startTime = System.nanoTime() / 1e9;
        follower.followTrajectory(trajectory, startTime);
        while (opModeIsActive() && follower.isFollowing(System.nanoTime() / 1e9)) {
            double time = System.nanoTime() / 1e9;
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d targetPose = trajectory.get(time - startTime);

            writer.put("time", time - startTime);
            writer.put("targetX", targetPose.getX());
            writer.put("targetY", targetPose.getY());
            writer.put("targetHeading", targetPose.getHeading());
            writer.put("currentX", currentPose.getX());
            writer.put("currentY", currentPose.getY());
            writer.put("currentHeading", currentPose.getHeading());
            writer.write();

            follower.update(currentPose, time);
            drive.updatePoseEstimate(time);
        }

        writer.close();
    }
}
