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
                .afterTime(0, new ParallelAction(arm.setAngle(90), arm.setExtension(7)))
                .splineToLinearHeading(new Pose2d(-40, -37, Math.toRadians(90)), Math.toRadians(-270))
                .stopAndAdd(arm.setAngle(40))
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