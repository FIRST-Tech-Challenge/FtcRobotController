package org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose =  new Pose2d (36, -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                
                
                
                .setReversed(false)
                .splineTo(new Vector2d(36, -12), Math.toRadians(0))//drive back to centre
                .splineTo(new Vector2d(59, -7), 0)//drive to stack
                /*
                .setReversed(true)
                .splineTo(new Vector2d(36, -12), Math.toRadians(180))//drive back to centre
                .splineTo(new Vector2d(9, -12), Math.toRadians(207))//drive to pole
                .setReversed(false)
                .splineTo(new Vector2d(36, -12), Math.toRadians(0))//drive back to centre
                .splineTo(new Vector2d(59, -7), 0)//drive to stack
                .setReversed(true)
                .splineTo(new Vector2d(36, -12), Math.toRadians(180))//drive back to centre
                .splineTo(new Vector2d(9, -12), Math.toRadians(207))//drive to poleSplineTest
                
                 */


                //.splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(90)), Math.toRadians(180))
                //.splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(30)),  Math.toRadians(210))
                .build();

        drive.followTrajectorySequence(trajSeq);


    }
}
