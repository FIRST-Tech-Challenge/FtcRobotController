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
                .splineTo(new Vector2d(8,36.7),Math.toRadians(90))
                .waitSeconds(1.5)
                .lineToY(58)
                .strafeTo(new Vector2d());



        Action test = trajectoryAction1.build();


        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        test
                )
        );


    }
}
