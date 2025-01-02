package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "opMode")
public class SimpleSampleHang extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        // right
        Trajectory trajectoryRight = drive.trajectoryBuilder(startPose)
                .strafeRight(36)
                .build();
        drive.followTrajectory(trajectoryRight);

        // forward
        Trajectory trajectoryForward = drive.trajectoryBuilder(trajectoryRight.end())
                .forward(24)
                .build();
        drive.followTrajectory(trajectoryForward);

        sleep(2000);

        // back
        Trajectory trajectoryBack = drive.trajectoryBuilder(trajectoryForward.end())
                .back(24)
                .build();
        drive.followTrajectory(trajectoryBack);

        // left
        Trajectory trajectoryLeft = drive.trajectoryBuilder(trajectoryBack.end())
                .strafeLeft(36)
                .build();
        drive.followTrajectory(trajectoryLeft);
    }
}
