package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.trajectory.PathTrajectorySegment;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.util.Collections;

@Autonomous
public class SplineFollowTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MyMecanumDrive drive = new MyMecanumDrive(hardwareMap);
        Path path = new Path(new QuinticSplineSegment(
                new QuinticSplineSegment.Waypoint(0.0, 0.0, 60.0, 0.0),
                new QuinticSplineSegment.Waypoint(40.0, 40.0, 60.0, 0.0)
        ));
        DriveConstraints baseConstraints = new DriveConstraints(20.0, 30.0, Math.PI / 2, Math.PI / 2);
        MecanumConstraints constraints = new MecanumConstraints(baseConstraints, drive.getTrackWidth(), drive.getWheelBase());
        Trajectory trajectory = new Trajectory(Collections.singletonList(new PathTrajectorySegment(path, constraints, 250)));
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                new PIDCoefficients(0, 0, 0),
                new PIDCoefficients(0, 0, 0),
                0.01797,
                0,
                0);
        File logRoot = LoggingUtil.getLogRoot(this);
        String prefix = "SplineFollowTest-" + System.currentTimeMillis();
        CSVWriter writer = new CSVWriter(new File(logRoot, prefix + ".csv"));

        waitForStart();

        double startTime = System.nanoTime() / 1e9;
        follower.followTrajectory(trajectory);
        while (opModeIsActive() && follower.isFollowing()) {
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

            follower.update(currentPose);
            drive.updatePoseEstimate();
        }
    }
}
