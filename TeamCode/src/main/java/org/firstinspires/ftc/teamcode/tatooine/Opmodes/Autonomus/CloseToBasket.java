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
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
import org.firstinspires.ftc.teamcode.tatooine.utils.unit.UnitConverter;
import org.firstinspires.ftc.teamcode.tatooine.utils.unit.unit;

@Autonomous(name = "CloseToBasket", group = "Autonomous")
public class CloseToBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Action trajectoryActionChosen;
        Pose2d beginPose = new Pose2d(6+(18.0/2.0) - 30, -(72-(18.0/2.0)), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false, false);
        wrist.setPosAng(0.5);
        Arm arm = new Arm(this, false);
        TrajectoryActionBuilder trajectoryRed = drive.actionBuilder(beginPose)
                .stopAndAdd(new InstantAction(wrist::stright))
                .afterTime(0, new ParallelAction(arm.setAngle(65), arm.setExtension(15)))
                .splineToLinearHeading(new Pose2d(0, -20-9, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(2)
//                .lineToY(-33)
//                .splineToLinearHeading(new Pose2d(-48.5, -38, Math.toRadians(90)), Math.toRadians(-270))
//                .stopAndAdd(new SequentialAction(intake.intake(), arm.setExtension(0), arm.setAngle(0), intake.setPowerAction(0)))
//                .waitSeconds(2)
//                .splineToLinearHeading(new Pose2d(-58,-60, Math.toRadians(45)), Math.toRadians(-90))
//                .stopAndAdd(new SequentialAction( arm.setAngle(90),  arm.setExtension(arm.getMaxExtend()),new InstantAction(wrist::openMin), outTake.outTake())))
                ;


        //.splineTo(new Vector2d(-36.70, -63.40), Math.toRadians(202.26));


                trajectoryActionChosen = trajectoryRed.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(arm.moveAngle(),
                        trajectoryActionChosen
                )
                ));


    }
}