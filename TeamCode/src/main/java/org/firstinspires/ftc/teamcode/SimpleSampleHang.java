package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.VerticalArm;

@Autonomous(name="Simple Sample Hang", group = "opMode")
public class SimpleSampleHang extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VerticalArm arm = new VerticalArm(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // close hand to start
        arm.closeHand();

        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        // right
        Trajectory trajectoryRight = drive.trajectoryBuilder(startPose)
                .strafeRight(38)
                .build();
        drive.followTrajectory(trajectoryRight);

        // forward
        Trajectory trajectoryForward = drive.trajectoryBuilder(trajectoryRight.end())
                .forward(24)
                .build();
        drive.followTrajectory(trajectoryForward);

        // raise arm
        arm.moveToHeight(26);
        sleep(2000);

        // move forward more
        trajectoryForward = drive.trajectoryBuilder(trajectoryForward.end())
                .forward(6)
                .build();
        drive.followTrajectory(trajectoryForward);

        // lower arm to hook
        arm.moveToHeight(22.5);
        sleep(1000);
        arm.openHand();
        sleep(500);

        // back
        Trajectory trajectoryBack = drive.trajectoryBuilder(trajectoryForward.end())
                .back(6)
                .build();
        drive.followTrajectory(trajectoryBack);

        // lower arm
        arm.moveToHeight(0);

        // back more
        trajectoryBack = drive.trajectoryBuilder(trajectoryBack.end())
                .back(24)
                .build();
        drive.followTrajectory(trajectoryBack);

        // left
        Trajectory trajectoryLeft = drive.trajectoryBuilder(trajectoryBack.end())
                .strafeLeft(38)
                .build();
        drive.followTrajectory(trajectoryLeft);
    }
}
