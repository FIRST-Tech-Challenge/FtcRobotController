package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

@Autonomous(name = "RedFarScore")

public class FarScore extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Action test;
        boolean isRed = CheckAlliance.isRed();
        Pose2d beginPose = new Pose2d(12.09+3.5, -59.84, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder trajectoryRed = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(10,-38),Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(8,-39),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-51,-58,Math.toRadians(0)),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0,-52),Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(44.3,-37.5,Math.toRadians(45)),Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-5,-44),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-51,-48),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(40,-56),Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(50,-50,Math.toRadians(30)),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-5,-44),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-46,-44),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-51,-48),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(45,-50),Math.toRadians(0));
        TrajectoryActionBuilder trajectoryBlue = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(10,-38),Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToSplineHeading(new Vector2d(8,-39),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-51,-58,Math.toRadians(0)),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0,-52),Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(44.3,-37.5,Math.toRadians(45)),Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-5,-44),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-51,-48),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(40,-56),Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(50,-50,Math.toRadians(30)),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-5,-44),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-46,-44),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-51,-48),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(45,-50),Math.toRadians(0));


        if (isRed) {
            test = trajectoryRed.build();
        }
        else {
            test = trajectoryBlue.build();
        }

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        test
                )
        );


    }
}