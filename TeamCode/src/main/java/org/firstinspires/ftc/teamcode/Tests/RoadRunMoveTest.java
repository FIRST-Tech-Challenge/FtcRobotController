package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//@Disabled
@Autonomous(name = "RoadRunMoveTest")
public class RoadRunMoveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, false);
        SampleMecanumDrive roadrun = new SampleMecanumDrive(this.hardwareMap);
        Pose2d startPose = new Pose2d(35.25, 57.75, Math.toRadians(270));
        roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        roadrun.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        TrajectorySequence trajSeq2 = roadrun.trajectorySequenceBuilder(new Pose2d(35.25,57.75, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(35.25, 35.25,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(13.75, 35.25,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(13.75, 57.75,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(35.25, 57.75,Math.toRadians(270)))
                .build();

        while (opModeIsActive()) {
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.followTrajectorySequence(trajSeq2);
            roadrun.update();
            telemetry.update();
            robot.update();
        }
    }
}
