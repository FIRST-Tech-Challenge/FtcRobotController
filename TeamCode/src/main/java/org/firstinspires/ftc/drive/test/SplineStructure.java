package org.firstinspires.ftc.drive.test;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineStructure extends LinearOpMode {

    public static TrajectorySequence myTrajectory(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequenceBuilder ret;
        drive.setPoseEstimate(startPose);

        ret = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(15, 25), 0)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(10, -10), 0)
                .waitSeconds(3)
                .turn(Math.toRadians(45))
                .forward(10)
                .strafeLeft(5)
                .turn(Math.toRadians(90))
                .strafeLeft(5)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(45)), 0)
                ;

        return ret.build();
    }

    public static TrajectorySequenceBuilder goToCorner(TrajectorySequenceBuilder x) {
        return x.splineToConstantHeading(new Vector2d(30, 30), 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence trajSeq = myTrajectory(drive, startPose);

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
}
