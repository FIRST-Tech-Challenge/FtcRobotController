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
                .splineTo(new Vector2d(10,-38),Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(8,-39),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-51,-53.5,Math.toRadians(0)),Math.toRadians(180));
//                .strafeTo(new Vector2d(-52.5,-54))
//                .strafeTo(new Vector2d(56.4, -54))
//                .turnTo(Math.toRadians(90))
//                .turnTo(Math.toRadians(180))
//                .strafeTo(new Vector2d(-52.5,-54))
//                .strafeTo(new Vector2d(56.4, -54))
//                .turnTo(Math.toRadians(90))
//                .turnTo(Math.toRadians(180))
//                .strafeTo(new Vector2d(-54.5,-54))
//                .strafeTo(new Vector2d(56.4, -52))
//                .turnTo(Math.toRadians(90))
//                .turnTo(Math.toRadians(180))
//                .strafeTo(new Vector2d(-54.5,-54))
//                .strafeTo(new Vector2d(56.4, -54))
//                .turnTo(Math.toRadians(90))
//                .turnTo(Math.toRadians(180));




        Action test = trajectoryAction1.build();


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        test
                )
        );


    }
}