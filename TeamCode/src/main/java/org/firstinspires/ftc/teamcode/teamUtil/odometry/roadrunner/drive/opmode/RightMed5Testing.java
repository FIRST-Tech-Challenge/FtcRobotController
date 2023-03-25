package org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RightMed5Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose =  new Pose2d (36, -64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(36, -12),  Math.toRadians(90))//drive forward
                .turn(Math.toRadians(-70))//turn
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(59, -10, 0))//drive to stack
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(20)))
                .lineToLinearHeading(new Pose2d(59, -10, 0))//drive to stack
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(20)))
                .lineToLinearHeading(new Pose2d(59, -10, 0))//drive to stack
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(20)))
                
                /*.splineTo(new Vector2d(36, -12), Math.toRadians(210))//drive back to centre
                .setReversed(false)
                .splineTo(new Vector2d(59, -7), 0)//drive to stack
                .setReversed(true)
                .splineTo(new Vector2d(36, -12), Math.toRadians(210))//drive back to centre
                .setReversed(false)
                .splineTo(new Vector2d(59, -7), 0)//drive to stack
                .setReversed(true)
                .splineTo(new Vector2d(36, -12), Math.toRadians(210))//drive back to centre
                
                 */


                //.splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(90)), Math.toRadians(180))
                //.splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(30)),  Math.toRadians(210))
                .build();

        drive.followTrajectorySequence(trajSeq);


    }
}
