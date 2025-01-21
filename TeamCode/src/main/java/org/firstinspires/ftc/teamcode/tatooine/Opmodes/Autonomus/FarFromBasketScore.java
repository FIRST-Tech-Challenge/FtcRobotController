package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

@Autonomous(name = "FarFromBasketScore", group = "Autonomous")

public class FarFromBasketScore extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = CheckAlliance.isRed();
        Action trajectoryActionChosen;
        Pose2d beginPose = new Pose2d(6+(18.0/2.0), -(72-(18.0/2.0)), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Wrist wrist = new Wrist(this, false);
        wrist.setPosAng(0.5);
        Arm arm = new Arm(this, false);
        TrajectoryActionBuilder trajectoryRed = drive.actionBuilder(beginPose)

                .stopAndAdd(new InstantAction(wrist::stright))
                .afterTime(0, new ParallelAction(arm.setAngle(90), arm.setExtension(5)))
                .splineToLinearHeading(new Pose2d(4, -24-9, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(arm.setAngle(50))
                //armActions
//                .splineToLinearHeading(new Pose2d(22, -45, Math.toRadians(45)), Math.toRadians(90))
//                //extend
//                .turnTo(Math.toRadians(345))
//                //arm up little
//                .splineToLinearHeading(new Pose2d(32, -42, Math.toRadians(45)), Math.toRadians(-90))
//                //arm down
//                .turnTo(Math.toRadians(345))
//                //arm up little
//                .splineToLinearHeading(new Pose2d(40, -42, Math.toRadians(45)), Math.toRadians(-90))
//                //arm down
//                .turnTo(345)
//                .turnTo(Math.toRadians(-90))
//                //arm up
//                //extend less maybe
//                //intake
//                .waitSeconds(3)
//                .splineToLinearHeading(new Pose2d(6, -42, Math.toRadians(90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //outtake
//                .strafeTo(new Vector2d(6, -43))
//                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //intake
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(6, -40, Math.toRadians(90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //outtake
//                .strafeTo(new Vector2d(6, -43))
//                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(-90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //intake
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(6,-43))
//                .splineToLinearHeading(new Pose2d(6, -40, Math.toRadians(90)), Math.toRadians(0))
//                //arm up
//                //extend
//                //outtake
//                .splineToLinearHeading(new Pose2d(60,-60, Math.toRadians(-45)), Math.toRadians(0))
        ;
        TrajectoryActionBuilder trajectoryBlue = drive.actionBuilder(beginPose);


        if (isRed) {
            trajectoryActionChosen = trajectoryRed.build();
        } else {
            trajectoryActionChosen = trajectoryBlue.build();
        }

        waitForStart();

        Actions.runBlocking(new ParallelAction(arm.moveAngle(),  trajectoryActionChosen
        ));


    }
}