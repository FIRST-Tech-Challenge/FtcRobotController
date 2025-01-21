package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
@TeleOp

public class FFBSNew  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean isRed = CheckAlliance.isRed();
        Pose2d beginPose = new Pose2d(6+(18.0/2.0), -(72-(18.0/2.0)), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Wrist wrist = new Wrist(this, false);
        wrist.setPosAng(0.5);
        Arm arm = new Arm(this, false);
        TrajectoryActionBuilder goScore = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(4, -24-9, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder goIntake = goScore.endTrajectory().fresh().splineToLinearHeading(new Pose2d(22, -45, Math.toRadians(45)), Math.toRadians(90))
                .turnTo(Math.toRadians(345));
        waitForStart();



        Actions.runBlocking(new ParallelAction(arm.moveAngle(),
                new SequentialAction(
                        new ParallelAction(
                        new InstantAction(wrist::stright),
                        goScore.build(),
                        arm.setAngle(50),
                        arm.setExtension(5)
                                ),
                arm.setAngle(50),
                new ParallelAction(
                goIntake.build(),
                        arm.setExtension(Arm.getMaxExtend()),arm.setAngle(0)
                        ))));


    }
}
