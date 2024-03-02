package org.firstinspires.ftc.teamcode.Testing.AutonRoutes.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Testing.AutonRoutes.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(name = "path 1", group = "drive")
public class FirstTrajectory extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-30.0, 50.0, Math.toRadians(270.0));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(24.0)
                .build();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
                .back(20.0)
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .splineTo(new Vector2d(-55, 30), Math.toRadians(270))
                .build();
        drive.followTrajectorySequence(trajSeq);
        drive.followTrajectorySequence(trajSeq2);
        drive.followTrajectorySequence(trajSeq3);


    }
}
