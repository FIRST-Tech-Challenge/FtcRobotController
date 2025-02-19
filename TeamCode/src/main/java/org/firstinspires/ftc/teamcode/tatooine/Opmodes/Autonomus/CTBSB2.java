package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Autonomus;

import androidx.annotation.NonNull;
import androidx.appcompat.widget.SearchView;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.tatooine.utils.States.Conts;
import org.opencv.core.Mat;

@Autonomous(name = "1+2", group = "Autonomous")
public class CTBSB2 extends LinearOpMode {



    @Override
    public void runOpMode(){
        Action trajectoryActionChosen;
        Pose2d beginPose = new Pose2d(6+(18.0/2.0) - 30, -(72-(18.0/2.0)), Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Wrist wrist = new Wrist(this, false);
        Intake intake = new Intake(this, false);
        Arm arm = new Arm(this, true);
        TrajectoryActionBuilder goScoreSpecimen = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-7, -31.5, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder goIntakeSample1 = goScoreSpecimen.endTrajectory().fresh()
                .strafeTo(new Vector2d(-8, -40 + 0.5))
                .splineToLinearHeading(new Pose2d(-49.5,-40 + 1.5,Math.toRadians(90)), Math.toRadians(180));
        TrajectoryActionBuilder goScoreSample1 = goIntakeSample1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-55.5,-58,Math.toRadians(45)), Math.toRadians(-90));
        TrajectoryActionBuilder goIntakeSample2 = goScoreSample1.endTrajectory().fresh()
                .turnTo(Math.toRadians(97.8));
                //                .strafeToLinearHeading(new Vector2d(-55, -58), Math.toRadians(96.2));
        TrajectoryActionBuilder goScoreSample2 = goIntakeSample2.endTrajectory().fresh()
                .turnTo(Math.toRadians(45));
       TrajectoryActionBuilder goPark = goScoreSample2.endTrajectory().fresh()
               .strafeToLinearHeading(new Vector2d(-52, 0),  Math.toRadians(0))
               .strafeToLinearHeading(new Vector2d(-22.5 , 0), Math.toRadians(0));
        waitForStart();
        Actions.runBlocking(new ParallelAction(
                        arm.moveExtend(),arm.moveAngle(),new SequentialAction(
                        new InstantAction(wrist::intakeFlat)
                                ,intake.intakeByTimer(1, 0.1)
//                        , arm.setAngle(90)
                        ,new ParallelAction(
                                new InstantAction(wrist::intakeFlat)
                                , arm.setAngle(Conts.angleScoreSpecimenHigh)
                                , arm.setExtend(14.3),
                                goScoreSpecimen.build())
//                    , arm.setAngle(45)
                        , arm.setExtend(0)
                        , new ParallelAction(goIntakeSample1.build() , new SequentialAction(intake.intakeByTimer(-1, 0.5)
                        ,intake.intakeByTimer(0, 0.1)))
                        , intake.intakeByTimer(1, 0.1)
                        , new InstantAction(wrist::intakeUp)
                        , new SleepAction(0.2)
                        , arm.setAngle(1)
                        , new SleepAction(1)
                        , new SequentialAction(arm.setAngle(10), new InstantAction(wrist::intakeFlat), intake.stop())
                        , goScoreSample1.build()
                        , new SequentialAction(
                        new ParallelAction(arm.setAngle(Conts.angleScoreSample), arm.setExtend(54))
                        , new InstantAction(wrist::doc),new SleepAction(0.35), intake.intakeByTimer(-1, 0.5))
                        , new SequentialAction(intake.stop(),new InstantAction(()-> wrist.intakeFlat()), arm.setExtend(0), arm.setAngle(Conts.angleDrive))
                        , goIntakeSample2.build()
                        , intake.intakeByTimer(1, 0.1)
                        , arm.setAngle(15)
                        , new InstantAction(wrist::intakeUp)
                        ,arm.setExtend(54)
                        ,new SleepAction(0.2)
                        ,arm.setAngle(0)
                        ,arm.setAngle(10)
                        ,intake.intakeByTimer(0, 0.1)
                        , arm.setExtend(0)
                        , goScoreSample2.build()
                        , new SequentialAction(
                        new ParallelAction(arm.setAngle(Conts.angleScoreSample), arm.setExtend(54))
                        , new InstantAction(wrist::doc),new SleepAction(0.35), intake.intakeByTimer(-1, 0.5))
                        , new SequentialAction(intake.stop(),new InstantAction(()-> wrist.intakeFlat()), arm.setExtend(40), arm.setAngle(50))
                        , new ParallelAction(goPark.build()
                        , arm.setExtend(10))
                )
                )
        );
    }

}