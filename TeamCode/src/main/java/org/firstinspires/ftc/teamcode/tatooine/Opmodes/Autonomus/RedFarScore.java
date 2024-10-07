package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
@Autonomous(name = "RedFarScore")

public class RedFarScore extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(12.09+3.5, -59.84, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder trajectoryAction1 = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(7.79, -36.70), Math.toRadians(90))
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(12.09+3.5, -56),Math.toRadians(90))
                .turnTo(0)
                .strafeToSplineHeading(new Vector2d(-50, -57.84), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(55, -53.5), Math.toRadians(70))
                .waitSeconds(1)
                .turnTo(Math.toRadians(70))
                .strafeToSplineHeading(new Vector2d(-54, -60), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(55, -52.8), Math.toRadians(105))
                .waitSeconds(1)
                .turnTo(Math.toRadians(105))
                .strafeToSplineHeading(new Vector2d(-54, -60), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(55, -52), Math.toRadians(90));

        Action test = trajectoryAction1.build();


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        test
                )
        );


    }
}
